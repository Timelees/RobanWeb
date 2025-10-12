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
        return False
    except Exception:
        return False

def execute_command(cmd):
    """执行命令并返回结果字典。

    - 对于包含 shell 特性的命令（如 &&、source、rosrun 等），使用 bash -lc 启动并返回 detached 信息。
    - 对于简单命令，使用 subprocess.run 同步执行并返回 stdout/stderr/return_code。
    """
    try:
        shell_indicators = ['&&', ';', '|', '>', '<', '$', '`', '"', "'", '*', '?', 'source']
        use_shell = any(ind in cmd for ind in shell_indicators) or 'rosrun' in cmd

        # 如果是启动 SLAM 的命令，生成 wrapper 脚本并用 nohup 后台启动，传递 DISPLAY/XAUTHORITY
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
                proc = subprocess.Popen(['/bin/bash', script_path], env=env, stdout=out_file, stderr=err_file, preexec_fn=os.setsid)
                pid = proc.pid
                try:
                    _proc_registry[pid] = proc
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
            proc = subprocess.Popen(['/bin/bash', '-lc', cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            pid = proc.pid
            try:
                _proc_registry[pid] = proc
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
        res = {
            "cmd": original_cmd,
            "return_code": rc,
            "stdout": out_s,
            "stderr": err_s,
            "duration": None,
            "pid": pid,
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
            pgid = os.getpgid(proc.pid)
            # 先发送 SIGINT（模拟 Ctrl+C），让 ROS 节点有机会优雅退出
            try:
                os.killpg(pgid, signal.SIGINT)
            except Exception:
                pass
            # wait up to 6s for graceful shutdown
            for _ in range(12):
                if proc.poll() is not None:
                    result['ok'] = True
                    result['killed'] = True
                    break
                time.sleep(0.5)
            # 如果仍未退出，尝试 SIGTERM
            if not result['ok']:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except Exception:
                    pass
                for _ in range(6):
                    if proc.poll() is not None:
                        result['ok'] = True
                        result['killed'] = True
                        break
                    time.sleep(0.5)
            # 最后使用 SIGKILL 强制结束
            if not result['ok']:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    pass
                result['ok'] = (proc.poll() is not None)
                result['killed'] = result['ok']
            try:
                del _proc_registry[pid]
            except Exception:
                pass
            return result
        else:
            # Try best-effort kill by pid
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                result['error'] = 'process not found'
                return result
            except Exception as e:
                result['error'] = str(e)
                return result
            # give it a moment and try SIGINT first
            try:
                os.kill(pid, signal.SIGINT)
            except Exception:
                pass
            time.sleep(0.5)
            try:
                os.kill(pid, 0)
                # still exists, try TERM then KILL
                try:
                    os.kill(pid, signal.SIGTERM)
                except Exception:
                    pass
                time.sleep(0.5)
                try:
                    os.kill(pid, signal.SIGKILL)
                except Exception:
                    pass
                result['ok'] = True
                result['killed'] = True
            except OSError:
                result['ok'] = True
                result['killed'] = True
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
        if not raw:
            rospy.logwarn("empty command payload")
            return

        # 尝试把 payload 当作 JSON 解析，若失败则视为普通命令字符串
        try:
            parsed = json.loads(raw)
            rospy.loginfo("Parsed JSON payload: %s", json.dumps(parsed, ensure_ascii=False))
            # 如果是 stop 请求，优先处理
            if isinstance(parsed, dict) and parsed.get('action') == 'stop':
                # 支持通过 pid 停止
                if 'pid' in parsed:
                    stop_res = _stop_proc_by_pid(parsed['pid'])
                    self.pub.publish(String(json.dumps({'action':'stop','result':stop_res})))
                    return
                # 支持通过 which: 'slam' 停止所有与 SLAM 相关的注册进程
                if parsed.get('which') == 'slam' or parsed.get('which') == 'SLAM':
                    results = []
                    # 遍历注册表，匹配 wrapper script 名称或 rosrun 出现
                    for pid, proc in list(_proc_registry.items()):
                        try:
                            cmdline_path = f'/proc/{pid}/cmdline'
                            matched = False
                            if os.path.exists(cmdline_path):
                                with open(cmdline_path, 'r') as cf:
                                    cmdline = cf.read()
                                    if 'start_slam_from_exec.sh' in cmdline or 'rosrun' in cmdline and 'SLAM' in cmdline:
                                        matched = True
                            if matched:
                                r = _stop_proc_by_pid(pid)
                                results.append(r)
                        except Exception as e:
                            results.append({'pid': pid, 'ok': False, 'error': str(e)})
                    self.pub.publish(String(json.dumps({'action':'stop','which':'slam','results':results})))
                    return

            if isinstance(parsed, dict):
                # 优先查找直接的 cmd 字段
                if 'cmd' in parsed and isinstance(parsed['cmd'], str):
                    cmd = parsed['cmd']
                # 支持 rosbridge/其它包装： {"msg": {"cmd": "..."}} 或 {"msg": {"data": "..."}}
                elif 'msg' in parsed and isinstance(parsed['msg'], dict):
                    inner = parsed['msg']
                    rospy.loginfo("Inner msg content: %s", json.dumps(inner, ensure_ascii=False))
                    if 'cmd' in inner and isinstance(inner['cmd'], str):
                        cmd = inner['cmd']
                    elif 'data' in inner and isinstance(inner['data'], str):
                        # 有些客户端会把内容放在 msg.data
                        cmd = inner['data']
                # 也有可能直接使用 data 字段作为 std_msgs/String 的内容
                elif 'data' in parsed and isinstance(parsed['data'], str):
                    cmd = parsed['data']
        except Exception as e:
            # 不是 JSON，就当作普通命令字符串
            rospy.logwarn("JSON parse failed: %s", str(e))
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
        self.executor.submit(self._run_and_publish, cmd)

    def _run_and_publish(self, cmd):
        res = execute_command(cmd)
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