# C++ 历史遗留问题汇总

> 本文整理 C++ 语言中公认的历史包袱与设计缺陷，并说明 UE5 如何解决或规避这些问题。

---

## 1. 命名误导

### `std::vector` — 叫"向量"，实为动态数组

`std::vector` 在数学上指向量，但其实现是连续内存的动态数组（类似 Java 的 `ArrayList`）。
真正的数学向量运算（点积、叉积）它并不支持。

```cpp
std::vector<int> v = {1, 2, 3};  // 实际是动态数组，支持随机访问、尾部 O(1) 插入
```

### `std::map` — 默认有序，多数场景不需要

`std::map` 基于红黑树，保证键有序，但大多数业务场景只需要查找，不需要有序遍历。
有序带来了 O(log n) 的额外开销，而 `std::unordered_map`（C++11）才是更常见需求的正确选择。

```cpp
std::map<int, int> m;          // 红黑树，有序，O(log n)
std::unordered_map<int, int> u; // 哈希表，无序，O(1) 均摊 —— 通常更快
```

**UE5 解法**：`TArray<T>` 替代 `std::vector`，名称直观语义清晰。`TMap<K,V>` 默认为哈希表（O(1) 均摊），`TSortedMap<K,V>` 用于需要有序遍历的场景，按需选择，不再"默认有序"。

---

## 2. 向后兼容包袱

C++ 承诺"不破坏已有代码"，导致废弃 API 无法删除，新旧并存：

| 旧（已废弃/移除） | 新（推荐） | 说明 |
|---|---|---|
| `auto_ptr` (C++11 废弃，C++17 移除) | `unique_ptr` | 独占所有权智能指针 |
| `std::bind1st` / `bind2nd` | `std::bind` / lambda | 函数适配器 |
| `register` 关键字 | 无（编译器自行优化） | C++17 起保留但无意义 |
| `throw(type)` 异常规范 | `noexcept` | 见第 8 节 |

**UE5 解法**：UE 自建类型系统（`TArray`、`TMap`、`FString` 等），不受 STL ABI 约束，可在大版本间做破坏性变更，彻底摆脱"不能删旧 API"的历史包袱。

---

## 3. 头文件 / 编译模型

`#include` 本质是文本替换，每个编译单元都要重新解析所有头文件，导致：

- 大型项目编译极慢（同一头文件被解析数百次）
- 头文件顺序影响编译结果
- 宏污染全局命名空间

**C++20 Modules** 引入了真正的模块系统，但截至 2025 年工具链支持仍不完整。

```cpp
// 传统方式：每个 .cpp 都重新解析 <vector>
#include <vector>

// C++20 模块（未来方向）
import std.vector;
```

**UE5 解法**：UnrealBuildTool 自动管理 PCH（预编译头）和 Unity Build（将多个 `.cpp` 合并为一个编译单元），大幅减少头文件重复解析，使大型项目编译速度可接受。

---

## 4. 隐式转换陷阱

C++ 允许大量静默的隐式转换，容易引入 bug：

```cpp
// 整数提升
char a = 200, b = 100;
int c = a + b;  // a、b 先提升为 int，结果 300，不溢出

// 窄化转换（C++11 列表初始化会报错，但赋值不会）
int x = 300;
char y = x;  // 静默截断，y = 44（未定义行为，实现相关）

// bool 转 int
bool flag = true;
int n = flag + 1;  // n = 2，合法但令人困惑

// 有符号/无符号混用
int i = -1;
unsigned u = 1;
if (i < u) { ... }  // 条件为 false！-1 被转为极大的无符号数
```

**UE5 解法**：`FString`、`FName`、`FText` 是强类型，互相之间不能隐式转换，必须显式调用 `FName::ToString()`、`FText::FromString()` 等。`TCHAR` 统一字符类型，避免 `char`/`wchar_t` 混用导致的隐式转换陷阱。

---

## 5. 未定义行为（UB）

C++ 标准将某些操作定义为"未定义行为"——编译器可以假设它们不会发生，
从而做出激进优化，导致程序行为完全不可预测，且**不报错**。

常见 UB：

```cpp
// 有符号整数溢出（无符号溢出是定义行为，会回绕）
int x = INT_MAX;
x += 1;  // UB！编译器可能优化掉溢出检查

// 空指针解引用
int* p = nullptr;
*p = 42;  // UB

// 数组越界
int arr[5];
arr[10] = 0;  // UB，不一定崩溃

// 使用未初始化变量
int n;
std::cout << n;  // UB，值不确定
```

工具：AddressSanitizer、UBSanitizer 可在运行时检测部分 UB。

**UE5 解法**：`check(expr)` 在 Debug/Development 模式下断言（违反则崩溃并打印调用栈），`ensure(expr)` 违反时打印日志但继续运行。UE 构建系统支持集成 AddressSanitizer 和 UBSanitizer，可在 CI 中捕获 UB。

---

## 6. 初始化规则混乱

C++ 有多种初始化方式，规则复杂且不一致：

```cpp
int a;          // 默认初始化：局部变量，值不确定（UB）
int b = 0;      // 拷贝初始化
int c(0);       // 直接初始化
int d{0};       // 列表初始化（C++11，推荐，禁止窄化）
int e{};        // 值初始化：零初始化为 0

struct Foo { int x; };
Foo f1;         // x 未初始化（局部）
Foo f2{};       // x = 0（值初始化）
Foo f3 = {};    // x = 0（同上）
```

**建议**：始终使用 `{}` 初始化，或显式赋初值。

**UE5 解法**：`UObject` 派生类由引擎负责零初始化，`UPROPERTY` 字段有默认值保证。非 UObject 结构体可用 `FMemory::Memzero(*this)` 显式清零，避免未初始化陷阱。

---

## 7. 宏的滥用

`#define` 宏无类型、无作用域、不参与重载，是预处理器的文本替换，极易出错：

```cpp
#define MAX(a, b) ((a) > (b) ? (a) : (b))

int x = 5;
int y = MAX(x++, 3);  // 展开为 ((x++) > (3) ? (x++) : (3))，x 被自增两次！

// 正确做法：constexpr 函数或模板
template<typename T>
constexpr T max_val(T a, T b) { return a > b ? a : b; }
```

历史代码库（包括 Windows SDK、部分 UE4 代码）大量使用宏，与现代 C++ 冲突。

**UE5 解法**：UE 宏有统一命名规范（`UE_LOG`、`UPROPERTY`、`UFUNCTION`、`UCLASS`），语义明确、有文档。现代 UE 代码逐步引入 `constexpr` 和模板替代部分宏，但宏仍是 UE 反射系统的核心机制。

---

## 8. 异常规范演变

```cpp
// C++98：动态异常规范（已在 C++17 移除）
void foo() throw(std::runtime_error);  // 只能抛出该类型
void bar() throw();                    // 不抛出异常

// C++11：noexcept（现代做法）
void baz() noexcept;           // 承诺不抛出，违反则 std::terminate
void qux() noexcept(false);    // 可能抛出
```

旧的 `throw()` 规范在运行时检查（有性能开销），`noexcept` 是编译期承诺，
允许编译器做更多优化（如移动语义的 `noexcept` 检查）。

**UE5 解法**：UE 默认在编译选项中禁用 C++ 异常（`-fno-exceptions`），用 `check`/`ensure`/`verify` 宏体系替代异常处理，避免异常带来的运行时开销和 ABI 复杂性。

---

## 9. `iostream` 性能问题

`std::cin` / `std::cout` 默认与 C 的 `stdio` 同步，导致性能远低于 `scanf` / `printf`：

```cpp
// 竞赛/高性能场景常见优化
std::ios::sync_with_stdio(false);
std::cin.tie(nullptr);
// 之后不能混用 printf/scanf
```

根本原因：`iostream` 为了与 C 库兼容，加了同步锁，牺牲了性能。

**UE5 解法**：引擎代码完全不使用 `iostream`，统一用 `UE_LOG(LogCategory, Verbosity, TEXT("..."))` 输出日志，`FString` 处理字符串格式化，性能和可控性均优于 `iostream`。

---

## 10. `std::string` 的 SSO 与 ABI 不兼容

各编译器对 `std::string` 的 Small String Optimization（SSO）实现不同：

| 实现 | SSO 阈值 | 内部布局 |
|---|---|---|
| libstdc++ (GCC) | 15 字节 | 内联缓冲区在对象内 |
| libc++ (Clang) | 22 字节 | 不同布局 |
| MSVC STL | 15 字节 | 又一种布局 |

**后果**：不同编译器编译的 `.dll` / `.so` 之间**不能直接传递 `std::string`**，
会导致内存布局错误或崩溃。这是 C++ ABI 不稳定问题的典型案例。

在 UE5 中，引擎使用 `FString`（基于 `TArray<TCHAR>`）而非 `std::string`，
部分原因正是为了规避跨模块 ABI 问题。

**UE5 解法**：`FString` 基于 `TArray<TCHAR>`，布局由 UE 自身定义，不依赖任何 STL 实现，跨模块传递安全。同时统一了字符编码（`TCHAR` 在 Windows 为 UTF-16），消除了 SSO 差异带来的 ABI 风险。

---

## 11. 多重继承菱形问题

C++ 支持多重继承，菱形继承需要虚继承（`virtual`）解决二义性，语法复杂，运行时有额外开销：

```cpp
struct A { int x; };
struct B : virtual A {};
struct C : virtual A {};
struct D : B, C {};  // D::x 唯一，但需要虚继承机制支撑
```

虚继承引入虚基类指针（vbptr），增加对象大小和访问间接层。

**UE5 解法**：`UObject` 体系采用单根继承，多重行为通过 `UInterface`（纯接口）实现，彻底规避菱形继承。`IInterface` 只允许声明纯虚函数，不携带数据成员。

---

## 12. 内存管理碎片化

`new`/`delete` 直接调用系统分配器，无法追踪、无法统计，大量小对象分配导致内存碎片：

```cpp
Foo* p = new Foo();   // 无法知道谁分配、何时释放
delete p;             // 忘记 delete → 泄漏；double delete → 崩溃
```

**UE5 解法**：`FMemory::Malloc`/`Free` 统一分配器，底层可替换（支持 mimalloc 等）。`UObject` 由 GC 管理，无需手动 `delete`。内存追踪工具（`-memreport`、Unreal Insights）可统计各类型内存占用。

---

## 13. 线程安全缺失

STL 容器（`std::vector`、`std::map` 等）不是线程安全的，并发读写需要外部加锁，但标准库没有提供配套同步原语：

```cpp
std::vector<int> v;
// 两个线程同时 push_back → 数据竞争，UB
```

**UE5 解法**：提供完整的并发原语：`FCriticalSection`（互斥锁）、`FRWLock`（读写锁）、`TAtomic<T>`（原子操作）、`TQueue<T>`（无锁队列）、`TLockFreePointerList`。`AsyncTask` 和 `TaskGraph` 系统封装了线程池调度。

---

## 14. 反射 / 序列化缺失

C++ 没有原生反射，无法在运行时查询类型信息、枚举字段、动态创建对象：

```cpp
// 标准 C++ 无法做到：
// - 按名称查找字段
// - 遍历所有 UPROPERTY
// - 将对象序列化为 JSON/二进制
```

**UE5 解法**：Unreal Header Tool（UHT）在编译前扫描 `UCLASS`/`UPROPERTY`/`UFUNCTION` 宏，自动生成 `.generated.h` 反射代码。运行时可通过 `UClass*`、`FProperty*` 枚举字段、动态调用函数，蓝图系统和序列化均基于此反射体系。

---

## 15. 字符编码混乱

`char` 的编码取决于平台和编译器设置，`wchar_t` 在 Windows 为 16 位、在 Linux 为 32 位，跨平台字符串处理极易出错：

```cpp
const char* s = "你好";      // 编码取决于源文件和编译器
const wchar_t* w = L"你好";  // Windows: UTF-16，Linux: UTF-32
```

**UE5 解法**：统一使用 `TCHAR`（Windows 下为 `wchar_t` UTF-16），字符串字面量用 `TEXT("...")` 宏包裹，`FString` 内部存储 `TArray<TCHAR>`，跨平台行为一致。

---

## 16. 全局对象构造顺序不确定

不同编译单元中的静态/全局对象初始化顺序未定义（Static Initialization Order Fiasco），依赖其他全局对象的初始化可能访问未构造的对象：

```cpp
// a.cpp
extern int g_value;
int g_double = g_value * 2;  // g_value 可能尚未初始化！

// b.cpp
int g_value = 42;
```

**UE5 解法**：引擎核心系统（日志、内存、GC）通过显式的 `FEngineLoop::PreInit` 按顺序初始化，避免依赖全局构造顺序。单例模式使用懒加载（`static` 局部变量或 `TLazySingleton<T>`），保证首次访问时才构造。

---

## 17. 模板错误信息难读

C++ 模板实例化失败时，编译器会输出冗长的错误信息，嵌套层级深时几乎无法阅读：

```
error: no matching function for call to 'std::vector<...>::push_back(...)'
  note: candidate: ...
  note: no known conversion from '...' to '...'
  [后续 50 行模板展开栈]
```

**UE5 解法**：UE 容器（`TArray`、`TMap` 等）减少深层模板嵌套，错误信息相对简洁。C++20 Concepts（UE5.3+ 部分支持）可在模板参数不满足约束时给出清晰的错误提示，而非展开整个实例化栈。

---

## 18. 缺乏垃圾回收

C++ 手动管理内存，忘记 `delete` 导致泄漏，过早 `delete` 导致悬空指针，智能指针（`shared_ptr`）的循环引用同样泄漏：

```cpp
auto a = std::make_shared<Node>();
auto b = std::make_shared<Node>();
a->next = b;
b->prev = a;  // 循环引用，引用计数永不归零 → 泄漏
```

**UE5 解法**：`UObject` 体系实现标记-清除 GC，引擎定期扫描所有 `UObject`，回收不可达对象，无需手动 `delete`。`UPROPERTY` 指针自动参与 GC 根集追踪，`TWeakObjectPtr` 用于弱引用（不阻止回收）。非 `UObject` 对象仍用 `TSharedPtr`/`TUniquePtr`（UE 版智能指针）管理。

---

## 参考

- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
- [cppreference.com](https://en.cppreference.com)
- Herb Sutter, *Exceptional C++* 系列
