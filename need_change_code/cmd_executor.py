#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import shlex
import subprocess
import threading
import json
import time
import os
import shutil
import signal
from concurrent.futures import ThreadPoolExecutor, as_completed

# 配置
EXEC_TOPIC = "/robot/exec_sh"
RESULT_TOPIC = "/robot/exec_sh_result"
MAX_WORKERS = 2
CMD_TIMEOUT = 30  # seconds
# 白名单（只允许这几类命令或脚本路径）
ALLOWED_PREFIXES = [
    "/home/robot/scripts/",   # 以脚本路径调用
    "roslaunch",
    "rosrun",
    "source",  # 如果必须用 source, 请用 wrapper 脚本更安全
]

def is_allowed(cmd):
    # 简单策略：按第一个 token 检查是否允许；更严格可用完整白名单匹配
    try:
        parts = shlex.split(cmd)
        if not parts:
            return False
        first = parts[0]
        # 允许绝对路径脚本或预定义命令前缀
        for p in ALLOWED_PREFIXES:
            if first.startswith(p) or first == p or first == 'rosrun' or first == 'roslaunch':
                return True
        # 允许启动当前用户家目录下的可执行脚本（例如 /home/lemon/move.sh）
        # 只允许绝对路径且文件存在并且可执行
        if first.startswith('/'):
            if os.path.exists(first) and os.access(first, os.X_OK):
                # 进一步限制：只允许当前用户家目录下的脚本或 explicitly allowed dirs
                user_home = os.path.expanduser('~')
                if first.startswith(user_home):
                    return True
        return False
        return False
    except Exception:
        return False

def execute_command(cmd, task=None):
    """执行命令并返回结果字典。

    - 对于包含 shell 特性的命令（如 &&、source、rosrun 等），使用 bash -lc 启动并返回 detached 信息。
    - 对于简单命令，使用 subprocess.run 同步执行并返回 stdout/stderr/return_code。
    """
    try:
        shell_indicators = ['&&', ';', '|', '>', '<', '$', '`', '"', "'", '*', '?', 'source']
        use_shell = any(ind in cmd for ind in shell_indicators) or 'rosrun' in cmd

        # 如果是启动 SLAM 的命令，生成 wrapper 脚本并用 nohup 后台启动，传递 DISPLAY/XAUTHORITY
        # 先检测是否为本地绝对路径可执行脚本（比如 /home/lemon/move.sh）
        try:
            first_token = shlex.split(cmd)[0]
        except Exception:
            first_token = ''

        # 如果命令第一个 token 是本地绝对路径且文件存在，优先以 detached 模式启动并记录到注册表。
        # 即使文件当前没有可执行权限，也尝试用 /bin/bash 启动，这样能保证我们能记录并后续停止。
        if first_token and first_token.startswith('/') and os.path.exists(first_token):
            # 直接以 detached 模式启动该脚本，并记录到注册表
            start = time.time()
            script_base = os.path.basename(first_token)
            out_path = os.path.join('/tmp', f"{script_base}.out")
            err_path = os.path.join('/tmp', f"{script_base}.err")
            out_file = None
            err_file = None
            try:
                out_file = open(out_path, 'a')
                err_file = open(err_path, 'a')
                # 如果文件可执行，尽量直接 exec；否则使用 /bin/bash 启动以避免权限或 shebang 问题
                try:
                    if os.access(first_token, os.X_OK):
                        proc = subprocess.Popen([first_token] + shlex.split(cmd)[1:], stdout=out_file, stderr=err_file, stdin=subprocess.DEVNULL, preexec_fn=os.setsid, env=os.environ.copy(), close_fds=True)
                    else:
                        rospy.loginfo("script %s not executable, launching via /bin/bash", first_token)
                        proc = subprocess.Popen(['/bin/bash', first_token] + shlex.split(cmd)[1:], stdout=out_file, stderr=err_file, stdin=subprocess.DEVNULL, preexec_fn=os.setsid, env=os.environ.copy(), close_fds=True)
                except OSError as oe:
                    # Errno 8: Exec format error — 常见于脚本缺少 shebang；回退用 /bin/bash 启动
                    if getattr(oe, 'errno', None) == 8:
                        rospy.logwarn('Exec format error for %s, retrying with /bin/bash', first_token)
                        proc = subprocess.Popen(['/bin/bash', first_token] + shlex.split(cmd)[1:], stdout=out_file, stderr=err_file, stdin=subprocess.DEVNULL, preexec_fn=os.setsid, env=os.environ.copy(), close_fds=True)
                    else:
                        raise
                pid = proc.pid
                try:
                    _proc_registry[pid] = proc
                    # 标记为 control 类型（由 move.sh 启动的本地控制脚本）
                    _proc_meta[pid] = {'cmd': cmd, 'which': 'control', 'script': first_token}
                    # 记录 task <-> pid 映射（可为 None）
                    if task:
                        _pid_to_task[pid] = task
                        _task_to_pids.setdefault(task, []).append(pid)
                except Exception:
                    pass
                duration = time.time() - start
                return {
                    'cmd': cmd,
                    'return_code': 0,
                    'stdout': '',
                    'stderr': '',
                    'duration': duration,
                    'pid': pid,
                    'detached': True,
                    'launched_script': first_token,
                    'out_log': out_path,
                    'err_log': err_path
                }
            except Exception as e:
                rospy.logwarn('failed to launch local script %s: %s', first_token, str(e))
                return {'cmd': cmd, 'return_code': -4, 'stdout': '', 'stderr': str(e), 'duration': 0}
            finally:
                # don't close out_file/err_file here because child process may still write to them
                try:
                    if out_file:
                        out_file.close()
                except Exception:
                    pass
                try:
                    if err_file:
                        err_file.close()
                except Exception:
                    pass

        if 'rosrun SLAM' in cmd or 'rosrun ORB_SLAM' in cmd or 'rosrun slam' in cmd.lower():
            # 优先使用当前用户的家目录下路径；允许通过环境变量 CMD_EXEC_SCRIPTS_DIR 覆盖
            default_dir = os.path.join(os.path.expanduser('~'), 'robot_exec_scripts')
            scripts_dir = os.environ.get('CMD_EXEC_SCRIPTS_DIR', default_dir)
            try:
                os.makedirs(scripts_dir, exist_ok=True)
            except Exception as e:
                rospy.logwarn('could not create scripts dir %s: %s; falling back to /tmp', scripts_dir, str(e))
                scripts_dir = '/tmp/robot_exec_scripts'
                try:
                    os.makedirs(scripts_dir, exist_ok=True)
                except Exception as e2:
                    rospy.logwarn('failed to create fallback scripts dir %s: %s', scripts_dir, str(e2))
            script_path = os.path.join(scripts_dir, 'start_slam_from_exec.sh')
            try:
                with open(script_path, 'w') as f:
                    f.write('#!/bin/bash\n')
                    # 直接写入要执行的命令；不要写 exec bash（不需要交互 shell）
                    f.write(cmd + '\n')
                os.chmod(script_path, 0o755)
                rospy.loginfo('wrote wrapper script: %s', script_path)
            except Exception as e:
                err_msg = f'failed to write wrapper script {script_path}: {e}'
                rospy.logwarn(err_msg)
                # 立即返回错误结果，避免继续尝试启动无法写入的脚本
                return {
                    'cmd': cmd,
                    'return_code': -3,
                    'stdout': '',
                    'stderr': err_msg,
                    'duration': 0,
                    'detected_issue': 'write_wrapper_failed'
                }

            env = os.environ.copy()
            display = os.environ.get('DISPLAY')
            xauth = os.environ.get('XAUTHORITY')
            # 如果当前环境没有 DISPLAY，尝试回退为 :0 并使用当前用户的 XAUTHORITY
            if display:
                env['DISPLAY'] = display
            else:
                # 试用常见的本地 display
                env['DISPLAY'] = os.environ.get('DISPLAY') or ':0'
                # 尝试找到用户的 .Xauthority 文件
                user_xauth = os.path.join(os.path.expanduser('~'), '.Xauthority')
                if os.path.exists(user_xauth):
                    env['XAUTHORITY'] = user_xauth
            if xauth:
                env['XAUTHORITY'] = xauth
            start = time.time()
            try:
                # 直接用 bash 执行脚本（在子进程组中），不使用 nohup/& 让 Popen 管理子进程
                out_file = open('/tmp/start_slam.out', 'a')
                err_file = open('/tmp/start_slam.err', 'a')
                proc = subprocess.Popen(['/bin/bash', script_path], env=env, stdout=out_file, stderr=err_file, stdin=subprocess.DEVNULL, preexec_fn=os.setsid, close_fds=True)
                pid = proc.pid
                try:
                    _proc_registry[pid] = proc
                    # 如果是通过 wrapper 启动 SLAM，标记为 slam
                    _proc_meta[pid] = {'cmd': cmd, 'which': 'slam', 'wrapper': script_path}
                    if task:
                        _pid_to_task[pid] = task
                        _task_to_pids.setdefault(task, []).append(pid)
                except Exception:
                    pass
                duration = time.time() - start
                return {
                    'cmd': cmd,
                    'return_code': 0,
                    'stdout': '',
                    'stderr': '',
                    'duration': duration,
                    'pid': pid,
                    'detached': True,
                    'wrapper_script': script_path
                }
            except Exception as e:
                rospy.logwarn('failed to launch wrapper script: %s', str(e))
                # fallback to normal shell launch below

        if use_shell:
            start = time.time()
            proc = subprocess.Popen(['/bin/bash', '-lc', cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL, preexec_fn=os.setsid, close_fds=True)
            pid = proc.pid
            try:
                _proc_registry[pid] = proc
                # 如果命令中包含 rosrun 且指向 SLAM，标记为 slam，方便停止
                if 'rosrun' in cmd and 'slam' in cmd.lower():
                    _proc_meta[pid] = {'cmd': cmd, 'which': 'slam'}
                else:
                    _proc_meta[pid] = {'cmd': cmd, 'which': None}
                if task:
                    _pid_to_task[pid] = task
                    _task_to_pids.setdefault(task, []).append(pid)
            except Exception:
                pass
            duration = time.time() - start
            return {
                'cmd': cmd,
                'return_code': 0,
                'stdout': '',
                'stderr': '',
                'duration': duration,
                'pid': pid,
                'task': task,
                'detached': True
            }

        # 简单命令，按原样同步执行并返回输出
        args = shlex.split(cmd)
        start = time.time()
        proc = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=CMD_TIMEOUT, check=False)
        duration = time.time() - start
        out = proc.stdout.decode('utf-8', errors='replace')
        err = proc.stderr.decode('utf-8', errors='replace')
        return {
            'cmd': cmd,
            'return_code': proc.returncode,
            'stdout': out,
            'stderr': err,
            'duration': duration
        }
    except subprocess.TimeoutExpired as e:
        return {'cmd': cmd, 'return_code': -1, 'stdout': '', 'stderr': 'timeout', 'duration': CMD_TIMEOUT}
    except Exception as e:
        return {'cmd': cmd, 'return_code': -1, 'stdout': '', 'stderr': str(e), 'duration': 0}


# 全局注册表，用于存储在 detached 模式下启动的 Popen 对象，key 为 pid
_proc_registry = {}
# 附加元数据：pid -> { 'cmd':..., 'which': 'control'|'slam'|None, ... }
_proc_meta = {}
# task -> list of pids
_task_to_pids = {}
# pid -> task
_pid_to_task = {}


def find_descendant_pids(root_pid):
    """返回 root_pid 的所有后代进程 pid 列表（不包含 root_pid 本身）。
    使用 /proc 遍历所有进程的 PPid 字段构建父子关系树，然后做 BFS 查找后代。
    """
    try:
        root_pid = int(root_pid)
    except Exception:
        return []
    children = {}
    try:
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            pid = int(entry)
            try:
                with open(f'/proc/{pid}/status', 'r') as f:
                    for line in f:
                        if line.startswith('PPid:'):
                            ppid = int(line.split()[1])
                            children.setdefault(ppid, []).append(pid)
                            break
            except Exception:
                continue
        # BFS
        res = []
        queue = [root_pid]
        while queue:
            cur = queue.pop(0)
            for c in children.get(cur, []):
                if c not in res:
                    res.append(c)
                    queue.append(c)
        return res
    except Exception:
        return []

def _watch_proc_and_publish(pid, pub, original_cmd):
    """在后台等待 pid 对应的 proc 完成，并发布最终结果到 pub。"""
    proc = _proc_registry.get(pid)
    if not proc:
        rospy.logwarn("watcher: proc with pid %s not found", pid)
        return
    try:
        out, err = proc.communicate()
        rc = proc.returncode
        out_s = out.decode('utf-8', errors='replace') if out else ''
        err_s = err.decode('utf-8', errors='replace') if err else ''
        task = _pid_to_task.get(pid)
        res = {
            "cmd": original_cmd,
            "return_code": rc,
            "stdout": out_s,
            "stderr": err_s,
            "duration": None,
            "pid": pid,
            "task": task,
            "detached": True,
            "finished": True
        }
        pub.publish(String(json.dumps(res)))
    except Exception as e:
        rospy.logwarn("watcher: error waiting proc %s: %s", pid, str(e))
    finally:
        try:
            del _proc_registry[pid]
        except Exception:
            pass
        # try:
        #     if pid in _proc_meta:
        #         del _proc_meta[pid]
        # except Exception:
        #     pass
        # 清理 pid <-> task 映射
        try:
            task = _pid_to_task.pop(pid, None)
            if task and task in _task_to_pids:
                try:
                    _task_to_pids[task] = [p for p in _task_to_pids[task] if p != pid]
                    if not _task_to_pids[task]:
                        del _task_to_pids[task]
                except Exception:
                    pass
        except Exception:
            pass


def _stop_proc_by_pid(pid):
    """Try to terminate a process (and its process group) by pid. Return dict result."""
    try:
        pid = int(pid)
    except Exception:
        return {"pid": pid, "ok": False, "error": "invalid pid"}

    proc = _proc_registry.get(pid)
    result = {"pid": pid, "ok": False, "killed": False}
    try:
        # If we have the Popen object, use its pid and wait
        if proc:
            # We used setsid when launching, so pgid should be the pid.
            # Try to get it, fallback to pid.
            try:
                pgid = os.getpgid(proc.pid)
            except Exception:
                pgid = proc.pid

            # 1. Send SIGINT to the entire process group
            try:
                os.killpg(pgid, signal.SIGINT)
            except Exception:
                pass
            
            # Wait a bit to see if parent exits
            parent_exited = False
            for _ in range(6): # 3 seconds
                if proc.poll() is not None:
                    parent_exited = True
                    break
                time.sleep(0.5)
            
            # 2. Send SIGTERM to the entire process group
            # Even if parent exited, children might be alive.
            try:
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                pass
                
            if not parent_exited:
                for _ in range(4): # 2 seconds
                    if proc.poll() is not None:
                        parent_exited = True
                        break
                    time.sleep(0.5)
            
            # 3. Send SIGKILL to the entire process group (Final Cleanup)
            try:
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass
            
            # Final check
            result['ok'] = (proc.poll() is not None)
            result['killed'] = result['ok']

            try:
                del _proc_registry[pid]
            except Exception:
                pass
            try:
                if pid in _proc_meta:
                    del _proc_meta[pid]
            except Exception:
                pass
            # 清理 pid->task 映射
            try:
                task = _pid_to_task.pop(pid, None)
                if task and task in _task_to_pids:
                    _task_to_pids[task] = [p for p in _task_to_pids[task] if p != pid]
                    if not _task_to_pids[task]:
                        del _task_to_pids[task]
            except Exception:
                pass
            return result
        else:
            # Try best-effort kill by pid. But first, attempt to find any descendant processes
            # (父进程可能已退出，但其子进程仍在运行)，先查找并杀死这些后代。
            try:
                desc = find_descendant_pids(pid)
            except Exception:
                desc = []

            if desc:
                rospy.loginfo("found descendant pids for %s: %s", pid, desc)
                # 尝试逐个终止后代进程
                any_ok = False
                for d in desc:
                    try:
                        # 先尝试按进程组杀掉（若存在组），否则按 pid
                        try:
                            pg = os.getpgid(d)
                            os.killpg(pg, signal.SIGINT)
                        except Exception:
                            try:
                                os.kill(d, signal.SIGINT)
                            except Exception:
                                pass
                        # 等待短暂时间观察是否退出
                        killed_this = False
                        for _ in range(6):
                            try:
                                os.kill(d, 0)
                                time.sleep(0.5)
                            except OSError:
                                killed_this = True
                                break
                        if not killed_this:
                            # 再尝试 TERM/KILL
                            try:
                                os.killpg(os.getpgid(d), signal.SIGTERM)
                            except Exception:
                                try:
                                    os.kill(d, signal.SIGTERM)
                                except Exception:
                                    pass
                            time.sleep(0.5)
                            try:
                                os.killpg(os.getpgid(d), signal.SIGKILL)
                            except Exception:
                                try:
                                    os.kill(d, signal.SIGKILL)
                                except Exception:
                                    pass
                            # 最后检测是否已退出
                            try:
                                os.kill(d, 0)
                            except OSError:
                                killed_this = True
                        if killed_this:
                            any_ok = True
                    except Exception as e:
                        rospy.logwarn("error trying to kill descendant %s: %s", d, str(e))
                result['ok'] = any_ok
                result['killed'] = any_ok
                if any_ok:
                    # 清理 _proc_registry/_proc_meta 中可能残留的 pid
                    for d in desc:
                        try:
                            if d in _proc_registry:
                                del _proc_registry[d]
                        except Exception:
                            pass
                        try:
                            if d in _proc_meta:
                                del _proc_meta[d]
                        except Exception:
                            pass
                return result

            # 若未找到后代或后代未能处理，回退到按 pid 杀死
            # 优先尝试杀掉进程组 (pgid == pid)，因为父进程可能已退出但子进程还在组内
            try:
                os.killpg(pid, signal.SIGINT)
            except Exception:
                pass
            
            try:
                os.kill(pid, signal.SIGINT)
            except Exception:
                pass
                
            time.sleep(0.5)
            
            try:
                os.killpg(pid, signal.SIGTERM)
            except Exception:
                pass

            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except Exception as e:
                result['error'] = str(e)

            time.sleep(0.5)
            
            try:
                os.killpg(pid, signal.SIGKILL)
            except Exception:
                pass
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass

            result['ok'] = True
            result['killed'] = True
            
            # 清理 meta
            try:
                if pid in _proc_meta:
                    del _proc_meta[pid]
            except Exception:
                pass
            
            return result
    except Exception as e:
        return {"pid": pid, "ok": False, "error": str(e)}

class ExecNode(object):
    def __init__(self):
        rospy.init_node('robot_exec_node')
        self.pub = rospy.Publisher(RESULT_TOPIC, String, queue_size=10)
        self.sub = rospy.Subscriber(EXEC_TOPIC, String, self.cb_exec, queue_size=10)
        self.executor = ThreadPoolExecutor(max_workers=MAX_WORKERS)
        rospy.loginfo("robot_exec_node ready, subscribe to %s", EXEC_TOPIC)

    def cb_exec(self, msg):
        # 打印接收到的完整 ROS 消息对象（便于调试字段名/包装情况）
        try:
            rospy.loginfo("Received ROS msg: %s", str(msg))
        except Exception:
            pass

        raw = msg.data.strip()
        rospy.loginfo("Received exec payload (raw): %s", raw)

        # 支持多种 payload 格式：
        # 1) 直接命令字符串："/path/to/script arg"
        # 2) JSON 字符串：{"cmd":"/path/to/script arg"}
        # 3) 嵌套格式：{"msg": {"cmd": "..."}} （一些客户端可能发这种）
        cmd = None
        task = None

        if not raw:
            rospy.logwarn("empty command payload")
            return

        # 尝试把 payload 当作 JSON 解析，若失败则视为普通命令字符串
        try:
            parsed = json.loads(raw)
            rospy.loginfo("Parsed JSON payload: %s", json.dumps(parsed, ensure_ascii=False) if not isinstance(parsed, str) else parsed)
            # 有些客户端会把 JSON 再作为字符串放入 data（双重序列化），例如: "{\"cmd\":\"...\"}"
            # 如果解析后是字符串，则尝试再解析一次
            if isinstance(parsed, str):
                try:
                    inner = json.loads(parsed)
                    rospy.loginfo("Unwrapped nested JSON payload: %s", json.dumps(inner, ensure_ascii=False))
                    parsed = inner
                except Exception:
                    # 解析失败则把 parsed 当作命令字符串
                    pass
            
            # 如果有 action 字段，优先处理 add/stop 请求
            # 如果 action=='add'，将 data 写入 sh_path 指定的脚本文件（创建文件并设置可执行权限）
            if isinstance(parsed, dict) and parsed.get('action') == 'add':
                # 支持多种 data 位置：parsed['msg']['data'] 或 parsed['data']
                sh_path = parsed.get('sh_path')
                data_content = None
                try:
                    if 'msg' in parsed and isinstance(parsed['msg'], dict) and 'data' in parsed['msg']:
                        data_content = parsed['msg']['data']
                    elif 'data' in parsed and isinstance(parsed['data'], str):
                        data_content = parsed['data']
                except Exception:
                    data_content = None

                # 未提供必要字段则返回错误信息
                if not sh_path:
                    self.pub.publish(String(json.dumps({'action':'add','ok':False,'error':'missing sh_path'})))
                    return
                if data_content is None:
                    self.pub.publish(String(json.dumps({'action':'add','ok':False,'error':'no data to write'})))
                    return

                # 写入脚本文件（覆盖写入），并设置为可执行。
                try:
                    dirpath = os.path.dirname(sh_path)
                    if dirpath and not os.path.exists(dirpath):
                        os.makedirs(dirpath, exist_ok=True)
                    # 写入文件；如果数据没有 shebang，则添加 bash shebang
                    try:
                        with open(sh_path, 'w') as sf:
                            if not isinstance(data_content, str):
                                data_content = str(data_content)
                            if not data_content.startswith('#!'):
                                sf.write('#!/bin/bash\n')
                            sf.write(data_content)
                            if not data_content.endswith('\n'):
                                sf.write('\n')
                        # 设置可执行权限（类似 sudo chmod +x）
                        os.chmod(sh_path, 0o755)
                        res = {'action':'add','sh_path':sh_path,'ok':True}
                        rospy.loginfo("Successfully added script: %s", sh_path)
                    except Exception as e:
                        res = {'action':'add','sh_path':sh_path,'ok':False,'error':str(e)}
                        rospy.logwarn("Failed to write script %s: %s", sh_path, str(e))
                    self.pub.publish(String(json.dumps(res)))
                    return
                except Exception as e:
                    self.pub.publish(String(json.dumps({'action':'add','ok':False,'error':str(e)})))
                    return

            # 如果是 stop 请求，优先处理
            if isinstance(parsed, dict) and parsed.get('action') == 'stop':
                # 支持通过 pid 停止
                if 'pid' in parsed:
                    stop_res = _stop_proc_by_pid(parsed['pid'])
                    self.pub.publish(String(json.dumps({'action':'stop','result':stop_res})))
                    return
                # 支持通过 which: 'slam' 停止所有与 SLAM 相关的注册进程
                # 支持通用的 which 停止，例如 which: 'slam' 或 which: 'control'
                if 'which' in parsed:
                    which_val = parsed.get('which')
                    script_path = parsed.get('script_path')
                    task_val = parsed.get('task')
                    results = []
                    rospy.loginfo("stop request received: which=%s, script_path=%s, task=%s", which_val, script_path, task_val)
                    # 优先使用 _proc_meta 中的标记匹配 —— 且如果提供了 script_path，则同时匹配 script 字段或 cmd 中包含路径
                    for pid, meta in list(_proc_meta.items()):
                        try:
                            matched = False
                            if meta and 'which' in meta and meta['which'] and which_val and meta['which'].lower() == which_val.lower():
                                if script_path:
                                    mscript = meta.get('script')
                                    mcmd = meta.get('cmd', '')
                                    if (mscript and mscript == script_path) or (script_path and script_path in mcmd):
                                        matched = True
                                else:
                                    matched = True
                            if matched:
                                rospy.loginfo("matched proc_meta pid=%s meta=%s", pid, json.dumps(meta, ensure_ascii=False))
                                r = _stop_proc_by_pid(pid)
                                rospy.loginfo("stop result for pid %s: %s", pid, json.dumps(r, ensure_ascii=False) if isinstance(r, dict) else str(r))
                                results.append(r)
                        except Exception as e:
                            rospy.logwarn("error stopping pid %s: %s", pid, str(e))
                            results.append({'pid': pid, 'ok': False, 'error': str(e)})

                    # 如果通过 meta 没有停止任何进程，尝试通过 task 映射停止（如果提供了 task）
                    if not results and task_val:
                        pids = list(_task_to_pids.get(task_val, []))
                        rospy.loginfo("attempting stop by task mapping, task=%s -> pids=%s", task_val, pids)
                        for pid in pids:
                            try:
                                r = _stop_proc_by_pid(pid)
                                results.append(r)
                            except Exception as e:
                                results.append({'pid': pid, 'ok': False, 'error': str(e)})

                    # 如果仍没有结果，回退到 /proc cmdline 模糊匹配（按 which 或 script_path）
                    if not results:
                        for pid, proc in list(_proc_registry.items()):
                            try:
                                cmdline_path = f'/proc/{pid}/cmdline'
                                matched = False
                                if os.path.exists(cmdline_path):
                                    with open(cmdline_path, 'r') as cf:
                                        cmdline = cf.read()
                                        # match by which value or by script_path substring
                                        if which_val and which_val.lower() in cmdline.lower():
                                            matched = True
                                        if script_path and script_path in cmdline:
                                            matched = True
                                if matched:
                                    rospy.loginfo("matched by /proc cmdline pid=%s cmdline=%s", pid, cmdline_path)
                                    r = _stop_proc_by_pid(pid)
                                    results.append(r)
                            except Exception as e:
                                results.append({'pid': pid, 'ok': False, 'error': str(e)})

                    # 最后尝试使用系统级搜索（pgrep -f script_path）来查找可能未在 _proc_registry 中的进程
                    if not results and script_path:
                        try:
                            rospy.loginfo("no matches in registry/meta/task; trying system pgrep for %s", script_path)
                            out = subprocess.check_output(['pgrep', '-f', script_path], stderr=subprocess.STDOUT)
                            pids = [int(x) for x in out.decode('utf-8').split() if x.strip().isdigit()]
                            rospy.loginfo("pgrep found pids: %s", pids)
                            for pid in pids:
                                try:
                                    r = _stop_proc_by_pid(pid)
                                    results.append(r)
                                except Exception as e:
                                    results.append({'pid': pid, 'ok': False, 'error': str(e)})
                        except subprocess.CalledProcessError as cpe:
                            # pgrep 返回非0说明未找到匹配，继续
                            rospy.loginfo("pgrep returned no matches for %s", script_path)
                        except Exception as e:
                            rospy.logwarn("error running pgrep for %s: %s", script_path, str(e))

                    rospy.loginfo("stop action results: %s", json.dumps(results, ensure_ascii=False))
                    self.pub.publish(String(json.dumps({'action':'stop','which':which_val,'results':results})))
                    return

                # 支持通过 task 标识停止（我们会在执行时记录 task -> pid）
                if 'task' in parsed:
                    task_val = parsed.get('task')
                    results = []
                    pids = list(_task_to_pids.get(task_val, []))
                    for pid in pids:
                        try:
                            r = _stop_proc_by_pid(pid)
                            results.append(r)
                        except Exception as e:
                            results.append({'pid': pid, 'ok': False, 'error': str(e)})
                    self.pub.publish(String(json.dumps({'action':'stop','task':task_val,'results':results})))
                    return

                # 解析出 cmd/task
                if isinstance(parsed, dict):
                    # 优先：直接的 cmd 字段或 script_path 字段
                    if 'cmd' in parsed and isinstance(parsed['cmd'], str):
                        cmd = parsed['cmd']
                        if 'task' in parsed and isinstance(parsed['task'], str):
                            task = parsed['task']
                    elif 'script_path' in parsed and isinstance(parsed['script_path'], str):
                        # 有些客户端会使用 script_path 字段来传递要执行的脚本完整路径
                        cmd = parsed['script_path']
                        if 'task' in parsed and isinstance(parsed['task'], str):
                            task = parsed['task']
                    # 支持 rosbridge/其它包装：{"msg": {...}}
                    elif 'msg' in parsed and isinstance(parsed['msg'], dict):
                        inner = parsed['msg']
                        rospy.loginfo("Inner msg content: %s", json.dumps(inner, ensure_ascii=False))
                        if 'cmd' in inner and isinstance(inner['cmd'], str):
                            cmd = inner['cmd']
                            if 'task' in inner and isinstance(inner['task'], str):
                                task = inner['task']
                        elif 'data' in inner and isinstance(inner['data'], str):
                            # inner.data 可能是 JSON 字符串或直接命令，尝试解析
                            try:
                                inner_parsed = json.loads(inner['data'])
                                if isinstance(inner_parsed, dict) and 'cmd' in inner_parsed:
                                    cmd = inner_parsed.get('cmd')
                                    if 'task' in inner_parsed:
                                        task = inner_parsed.get('task')
                                else:
                                    cmd = inner['data']
                            except Exception:
                                cmd = inner['data']
                    # 也有可能直接使用 data 字段作为 std_msgs/String 的内容
                    elif 'data' in parsed and isinstance(parsed['data'], str):
                        try:
                            data_parsed = json.loads(parsed['data'])
                            if isinstance(data_parsed, dict) and 'cmd' in data_parsed:
                                cmd = data_parsed.get('cmd')
                                if 'task' in data_parsed:
                                    task = data_parsed.get('task')
                            else:
                                cmd = parsed['data']
                        except Exception:
                            cmd = parsed['data']
                elif isinstance(parsed, str):
                    # 解析后仍为字符串，直接当作命令
                    cmd = parsed
        except Exception as e:
            # 如果直接解析失败，尝试从 raw 中提取可能被转义或包裹的 JSON 子串并解析（例如客户端把 JSON 放在字符串里）
            rospy.logwarn("JSON parse failed: %s; trying to extract JSON substring", str(e))
            parsed = None
            try:
                first = raw.find('{')
                last = raw.rfind('}')
                if first != -1 and last != -1 and last > first:
                    candidate = raw[first:last+1]
                    try:
                        parsed = json.loads(candidate)
                        rospy.loginfo("Extracted JSON payload: %s", json.dumps(parsed, ensure_ascii=False))
                    except Exception as e2:
                        rospy.logwarn("extracted substring is not valid JSON: %s", str(e2))
            except Exception:
                parsed = None

            if parsed is None:
                # 如果还是没有解析到 JSON，尝试用正则从 raw 中抽取 action/sh_path/data（处理被当作字符串包裹或转义的情况）
                try:
                    action_match = re.search(r'"action"\s*:\s*"([^"\\]+)"', raw)
                    shpath_match = re.search(r'"sh_path"\s*:\s*"([^"\\]+)"', raw)
                    # data 可能包含转义字符，尽量捕获所有转义内容
                    data_match = re.search(r'"data"\s*:\s*"((?:\\.|[^"\\])*)"', raw)
                    action_val = action_match.group(1) if action_match else None
                    sh_path = shpath_match.group(1) if shpath_match else None
                    data_content = None
                    if data_match:
                        # data_match.group(1) 是未解码的 JSON 字符串片段，要用 json.loads 去解码转义
                        try:
                            data_content = json.loads('"' + data_match.group(1) + '"')
                        except Exception:
                            # 解码失败则直接使用原始捕获内容（可能包含转义）
                            data_content = data_match.group(1)

                    if action_val and action_val.lower() == 'add':
                        # 如果找到了 add 指令，则按 add 流程写文件（与上面相同的写入逻辑）
                        if not sh_path:
                            self.pub.publish(String(json.dumps({'action':'add','ok':False,'error':'missing sh_path'})))
                            return
                        if data_content is None:
                            self.pub.publish(String(json.dumps({'action':'add','ok':False,'error':'no data to write'})))
                            return
                        try:
                            dirpath = os.path.dirname(sh_path)
                            if dirpath and not os.path.exists(dirpath):
                                os.makedirs(dirpath, exist_ok=True)
                            with open(sh_path, 'w') as sf:
                                if not isinstance(data_content, str):
                                    data_content = str(data_content)
                                if not data_content.startswith('#!'):
                                    sf.write('#!/bin/bash\n')
                                sf.write(data_content)
                                if not data_content.endswith('\n'):
                                    sf.write('\n')
                            os.chmod(sh_path, 0o755)
                            res = {'action':'add','sh_path':sh_path,'ok':True}
                        except Exception as e:
                            res = {'action':'add','sh_path':sh_path,'ok':False,'error':str(e)}
                        self.pub.publish(String(json.dumps(res)))
                        return
                except Exception:
                    pass

                # 仍然不是 JSON，就当作普通命令字符串
                cmd = raw

        # 如果解析出来仍为空，退出
        if not cmd:
            rospy.logwarn("could not extract command from payload: %s", raw)
            return

        rospy.loginfo("Parsed exec command: %s", cmd)

        if not is_allowed(cmd):
            rospy.logwarn("command not allowed: %s", cmd)
            result = {"cmd":cmd, "return_code": -2, "stdout":"", "stderr":"command not allowed"}
            self.pub.publish(String(json.dumps(result)))
            return
        # 提交线程池异步执行
        self.executor.submit(self._run_and_publish, cmd, task)

    def _run_and_publish(self, cmd, task=None):
        res = execute_command(cmd, task=task)
        self.pub.publish(String(json.dumps(res)))
        # 如果是 detached 的 shell 启动，需要在后台线程中等待并回收子进程，随后发布最终结果
        if isinstance(res, dict) and res.get('detached') and 'pid' in res:
            pid = res.get('pid')
            # 启动 watcher 线程来等待 proc 完成并发布最终结果
            t = threading.Thread(target=_watch_proc_and_publish, args=(pid, self.pub, cmd), daemon=True)
            t.start()

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = ExecNode()
    node.spin()