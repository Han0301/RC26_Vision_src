# 为什么 `GetInstance` 是 `static` 且返回 `&`（引用）

> 分析这条声明：
> ```cpp
> static Ten_serial& GetInstance(const std::string& port = "/dev/ttyACM0",
>                                 const size_t& serial_baud = 115200);
> ```

---

## 1. `static` 的作用

### 1.1 `static` 修饰成员函数意味着什么

```cpp
class Ten_serial {
public:
    static Ten_serial& GetInstance(...);
    //   ^^^^^^
};
```

| 特性 | 普通成员函数 | 静态成员函数 |
|:----:|:-----------:|:-----------:|
| 调用方式 | `obj.func()` | `ClassName::func()` |
| 需要对象才能调用 | ✅ 必须有对象 | ❌ **不需要对象** |
| 有 `this` 指针 | ✅ 有 | ❌ **没有** |
| 能访问私有成员 | ✅ | ✅ **可以**（作为类成员） |

### 1.2 为什么单例必须用 `static`？

单例的核心矛盾：**你还没有对象的时候，怎么拿到那个唯一的对象？**

```cpp
// 如果 GetInstance 不是 static：
Ten_serial serial;                 // ❌ 构造函数是 private
serial.GetInstance(...);           // ❌ 死循环：需要对象→调函数→拿对象

// 作为 static 函数：
Ten::Ten_serial::GetInstance(...); // ✅ 不需要对象，直接用类名调用
```

**`static` 让你在"没有任何 `Ten_serial` 对象存在时"就能调用这个函数。**

### 1.3 `static` 函数如何创建对象？

```cpp
Ten_serial& Ten_serial::GetInstance(...)
{
    // 静态局部变量——生命周期贯穿整个程序
    static std::unique_ptr<Ten_serial> ten_serial = nullptr;

    std::call_once(serial_flag_, [port, serial_baud]() {
        ten_serial = create(port, serial_baud);  // create 是静态成员
        // create 可以访问 private 构造函数 → new Ten_serial(...)
    });

    return *ten_serial;
}
```

调用链：

```
GetInstance (static) → create (static) → new Ten_serial(port, baud) (private)
                                                      ↑
                                              静态成员可以访问私有成员
```

---

## 2. `&`（引用返回值）的作用

### 2.1 如果返回的是值（不是引用）

```cpp
// 假如这样写：
static Ten_serial GetInstance(...);  // 返回值，不是引用
```

调用方：

```cpp
auto serial = Ten::Ten_serial::GetInstance();  // 发生拷贝！
```

每次调用都会：

1. 函数内部拿到唯一的那个 `Ten_serial` 对象
2. **调用拷贝构造函数**，复制出一个全新的对象
3. 返回这个副本

后果：

| 问题 | 原因 |
|:----:|------|
| 单例被打破 | 每次调用都复制，存在多个对象 |
| 串口资源混乱 | 副本中的 `serial_` 共享同一个 fd？语义不清 |
| 编译错误 | 拷贝构造已被 `= delete` |

### 2.2 返回引用的好处

```cpp
static Ten_serial& GetInstance(...);  // 返回引用
```

调用方：

```cpp
auto& serial = Ten::Ten_serial::GetInstance();  // 零拷贝
```

| 特性 | 返回值 (`Ten_serial`) | 返回引用 (`Ten_serial&`) |
|:----:|:---:|:---:|
| 是否复制 | ✅ 每次调用都拷贝 | ❌ **零拷贝** |
| 是否保证唯一 | ❌ 每个调用方拿到不同副本 | ✅ **所有人拿到同一个对象** |
| 能否修改状态 | ❌ 改的是副本 | ✅ **改的就是原对象** |
| 需拷贝构造 | ✅ 需要（但已被 delete） | ❌ 不需要 |

### 2.3 类比

```
返回值 ≈ 复印机：图书馆的书（原对象）复印一份给你
引用   ≈ 导航：告诉你书在第 3 排书架上，你自己去看

对于串口这种硬件资源，你必须操作原件，不能操作复印件。
```

---

## 3. 两个设计合在一起看

```cpp
static Ten_serial& GetInstance(...);
// ^^^^^^        ^
// ①             ②
```

| 关键词 | 解决什么问题 | 没有它会怎样 |
|--------|-----------|-------------|
| ① `static` | 还没有对象时就能调用此函数 | 无法获取第一个对象（死锁） |
| ② `&` | 返回的是同一个对象，不复制 | 每个调用方拿到不同副本，单例失效 |

### 调用示意图

```cpp
// 第 1 次调用 —— 没有对象
auto& s1 = Ten::Ten_serial::GetInstance();
//          ↑ static: 允许在没有对象时调用
//          ↑ &:      s1 直接引用函数内部的那个对象

// 第 2 次调用 —— 对象已经存在
auto& s2 = Ten::Ten_serial::GetInstance();
//          ↑ static: 仍然可以调用
//          ↑ &:      s2 引用和 s1 同一个对象

// 验证
std::cout << (&s1 == &s2);  // 输出 1（true，是同一个对象）
```

---

## 4. 对比：函数内 `static` 和类 `static`

这里有两种 `static`，不要混淆：

```cpp
class Ten_serial {
public:
    // ① 类静态成员函数：属于类，不属于对象
    static Ten_serial& GetInstance(...);
};

// 在实现中：
Ten_serial& Ten_serial::GetInstance(...)
{
    // ② 局部静态变量：函数返回后不销毁
    static std::unique_ptr<Ten_serial> ten_serial = nullptr;
    //   ^^^^^^
    return *ten_serial;
}
```

| `static` 位置 | 作用域 | 生命周期 | 作用 |
|:----------:|--------|----------|------|
| ① 类成员函数 | 整个类 | 整个程序 | 没有对象也能调用 |
| ② 函数局部变量 | 仅在 GetInstance 内 | 整个程序 | 函数返回后不销毁，保持单例对象 |

---

## 5. 类比：银行保险箱

```
场景：银行只有一个 VIP 保险箱，你需要凭证才能打开

static ≈ 银行的 24 小时服务电话
         你不需要先"进入银行"就能打电话询问

&     ≈ 他们告诉你保险箱的编号，而不是把保险箱搬到你家里
         你每次来都看同一个保险箱，而不是每人发一个

所以：
  Ten::Ten_serial::GetInstance()
  ↑ 打电话（static）              ↑ 告诉我编号（&）
  → 我每次都去同一个保险箱
  → 里面放的串口数据始终一致
```

---

## 6. 一句话总结

> **`static` 让你在没有任何对象时就能调用入口函数；`&` 保证你拿到的不是复印件而是原件本身。二者共同构成了单例模式的"唯一入口 + 唯一实例"的完整语义。**
