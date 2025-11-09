/*使得Qt的MOC系统正确处理taskmanager的类*/

#include "taskmanager.h"
#include <QtCore/qmetatype.h>

#if __has_include(<QtCore/qtmochelpers.h>)
#include <QtCore/qtmochelpers.h>
#else
QT_BEGIN_MOC_NAMESPACE
#endif

#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'taskmanager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.5.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSTaskENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSTaskENDCLASS = QtMocHelpers::stringData(
    "Task",
    ""
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSTaskENDCLASS_t {
    uint offsetsAndSizes[2];
    char stringdata0[5];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSTaskENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSTaskENDCLASS_t qt_meta_stringdata_CLASSTaskENDCLASS = {
    {
        QT_MOC_LITERAL(0, 4)   // "Task"
    },
    "Task"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSTaskENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

Q_CONSTINIT const QMetaObject Task::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CLASSTaskENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSTaskENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSTaskENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<Task, std::true_type>
    >,
    nullptr
} };

void Task::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    (void)_o;
    (void)_id;
    (void)_c;
    (void)_a;
}

namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSTaskManagerENDCLASS_t {};
static constexpr auto qt_meta_stringdata_CLASSTaskManagerENDCLASS = QtMocHelpers::stringData(
    "TaskManager",
    "taskAdded",
    "",
    "Task*",
    "task",
    "taskRemoved",
    "index",
    "taskSelected",
    "taskExecuted",
    "scriptPath",
    "onAddTaskClicked",
    "onImportTaskClicked",
    "onRunTaskClicked",
    "onTaskItemDoubleClicked",
    "QListWidgetItem*",
    "item",
    "showTaskContextMenu",
    "pos"
);
#else  // !QT_MOC_HAS_STRING_DATA
struct qt_meta_stringdata_CLASSTaskManagerENDCLASS_t {
    uint offsetsAndSizes[36];
    char stringdata0[12];
    char stringdata1[10];
    char stringdata2[1];
    char stringdata3[6];
    char stringdata4[5];
    char stringdata5[12];
    char stringdata6[6];
    char stringdata7[13];
    char stringdata8[13];
    char stringdata9[11];
    char stringdata10[17];
    char stringdata11[20];
    char stringdata12[17];
    char stringdata13[24];
    char stringdata14[17];
    char stringdata15[5];
    char stringdata16[20];
    char stringdata17[4];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_CLASSTaskManagerENDCLASS_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_CLASSTaskManagerENDCLASS_t qt_meta_stringdata_CLASSTaskManagerENDCLASS = {
    {
        QT_MOC_LITERAL(0, 11),  // "TaskManager"
        QT_MOC_LITERAL(12, 9),  // "taskAdded"
        QT_MOC_LITERAL(22, 0),  // ""
        QT_MOC_LITERAL(23, 5),  // "Task*"
        QT_MOC_LITERAL(29, 4),  // "task"
        QT_MOC_LITERAL(34, 11),  // "taskRemoved"
        QT_MOC_LITERAL(46, 5),  // "index"
        QT_MOC_LITERAL(52, 12),  // "taskSelected"
        QT_MOC_LITERAL(65, 12),  // "taskExecuted"
        QT_MOC_LITERAL(78, 10),  // "scriptPath"
        QT_MOC_LITERAL(89, 16),  // "onAddTaskClicked"
        QT_MOC_LITERAL(106, 19),  // "onImportTaskClicked"
        QT_MOC_LITERAL(126, 16),  // "onRunTaskClicked"
        QT_MOC_LITERAL(143, 23),  // "onTaskItemDoubleClicked"
        QT_MOC_LITERAL(167, 16),  // "QListWidgetItem*"
        QT_MOC_LITERAL(184, 4),  // "item"
        QT_MOC_LITERAL(189, 19),  // "showTaskContextMenu"
        QT_MOC_LITERAL(209, 3)   // "pos"
    },
    "TaskManager",
    "taskAdded",
    "",
    "Task*",
    "task",
    "taskRemoved",
    "index",
    "taskSelected",
    "taskExecuted",
    "scriptPath",
    "onAddTaskClicked",
    "onImportTaskClicked",
    "onRunTaskClicked",
    "onTaskItemDoubleClicked",
    "QListWidgetItem*",
    "item",
    "showTaskContextMenu",
    "pos"
};
#undef QT_MOC_LITERAL
#endif // !QT_MOC_HAS_STRING_DATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSTaskManagerENDCLASS[] = {

 // content:
      11,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   62,    2, 0x06,    1 /* Public */,
       5,    1,   65,    2, 0x06,    3 /* Public */,
       7,    1,   68,    2, 0x06,    5 /* Public */,
       8,    1,   71,    2, 0x06,    7 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      10,    0,   74,    2, 0x0a,    9 /* Public */,
      11,    0,   75,    2, 0x0a,   10 /* Public */,
      12,    0,   76,    2, 0x0a,   11 /* Public */,
      13,    1,   77,    2, 0x0a,   12 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,    9,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 14,   15,

 // methods: parameters
    QMetaType::Void, QMetaType::QPoint,   17,

       0        // eod
};

Q_CONSTINIT const QMetaObject TaskManager::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CLASSTaskManagerENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSTaskManagerENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSTaskManagerENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<TaskManager, std::true_type>,
        // method 'taskAdded'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<Task *, std::false_type>,
        // method 'taskRemoved'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'taskSelected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<Task *, std::false_type>,
        // method 'taskExecuted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'onAddTaskClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onImportTaskClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onRunTaskClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onTaskItemDoubleClicked'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QListWidgetItem *, std::false_type>,
        // method 'showTaskContextMenu'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QPoint &, std::false_type>
    >,
    nullptr
} };

void TaskManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TaskManager *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->taskAdded((*reinterpret_cast< std::add_pointer_t<Task*>>(_a[1]))); break;
        case 1: _t->taskRemoved((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 2: _t->taskSelected((*reinterpret_cast< std::add_pointer_t<Task*>>(_a[1]))); break;
        case 3: _t->taskExecuted((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 4: _t->onAddTaskClicked(); break;
        case 5: _t->onImportTaskClicked(); break;
        case 6: _t->onRunTaskClicked(); break;
        case 7: _t->onTaskItemDoubleClicked((*reinterpret_cast< std::add_pointer_t<QListWidgetItem*>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (TaskManager::*)(Task * );
            if (_t _q_method = &TaskManager::taskAdded; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (TaskManager::*)(int );
            if (_t _q_method = &TaskManager::taskRemoved; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (TaskManager::*)(Task * );
            if (_t _q_method = &TaskManager::taskSelected; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (TaskManager::*)(const QString & );
            if (_t _q_method = &TaskManager::taskExecuted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject *TaskManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TaskManager::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSTaskManagerENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int TaskManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void TaskManager::taskAdded(Task * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void TaskManager::taskRemoved(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void TaskManager::taskSelected(Task * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void TaskManager::taskExecuted(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP