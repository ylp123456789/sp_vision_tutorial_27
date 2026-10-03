# 项目理解报告

请尽量使用自己的语言回答以下问题。可以引用少量关键代码或伪代码，但不要只粘贴实现。
完成一节后删除该节末尾的待填写标记；本地检查会拒绝仍有未完成章节的报告。

## 1. 图像生命周期与所有权

解释本项目中图像源为什么会复用缓冲区，以及 `cv::Mat` 的普通复制对底层像素数据
意味着什么。说明你的修改让一个 `Frame` 在进入队列后拥有什么，并解释为何后续读取
不会再改变它。

这里图像源ImageSequenceSource内部有一个buffer_缓冲区用于读取图片，为了减少内存分配开销，它会复用这块缓冲区：每次调用next()读取新图片时，直接在
一个cv::Mat缓冲区里覆盖像素数据，而不是每次都新建内存。

同时cv::Mat采用浅拷贝机制：普通赋值或者拷贝Frame对象时，只会复制Mat的头部信息（宽高、通道、指针），不会复制底层像素数组，新旧两个Mat会指向同一块像素内存。原始代码直接queue_.push(frame)，队列里保存的Frame和producer本地frame共享图像内存；producer下一次调用source_->next(frame)复用buffer，就会覆盖队列中帧的像素，所以当worker再次拿到frame时，计算出来的指纹和原来不一样，导致校验和不匹配，报corrupted错误。

我把它修改为queue_.push(std::move(frame))，用移动把producer端的Frame内部cv::Mat的底层像素数据的所有权转移到队列中的Frame。移动之后producer本地的frame.image变成空矩阵，不再持有图像内存。队列中的Frame独立拥有这份像素数据，producer后续复用缓冲区读取新图像，只会修改本地空frame或者新分配的buffer，不会再改动已经入队的帧内容，保证worker拿到的图像不会被外部篡改。

## 2. 并发处理与恰好一次

结合 `BlockingQueue` 的 `push`、`pop` 和 `close` 行为，解释多个 worker 如何分工。
为什么你的实现既不会漏掉已经入队的帧，也不会重复处理同一帧？输入耗尽时，正在等待
以及仍在处理数据的 worker 分别会怎样？

BlockingQueue的核心行为有以下几个：
1.`push`：上锁后把元素移入队列，调用`notify_one`唤醒一个等待的 worker；队列 close 之后 push 直接丢弃元素。这里主要通过if判断closed_的值是否为true，是就return。
2.`pop`：这里主要是有unique_lock，并且调用`wait`阻塞休眠，等待条件（队列非空/队列关闭）满足；唤醒后如果队列不为空，取出元素给调用方，返回true；如果队列空且队列关闭的话，就返回false。
3.`close`：让队列变为关闭，然后唤醒所有正在wait的worker。告诉它们：队列关闭了，剩下的frame处理完就可以下班。

多个worker的分工如下：这里加了条件变量用的锁，让他们共用同一个阻塞队列，所以队列是线程且安全的。不重不漏的原因如下:每当队列中有帧，`pop`只会取出队列头部的一个Frame，取出之后队列内部会把该元素移除。队列元素一旦被worker取出，其他worker无法再拿到同一个帧，天然保证同一帧最多被一个worker处理，不会重复处理。producer 生产的帧全部 push 进入队列之后，队列内部保存着全部待处理帧，只要还在队列里，就不会丢失，worker 会持续 pop 取出，保证不会漏帧。所以不重不漏。

输入全部耗尽后producer调用`queue_.close()`，他们的情况如下：
1.正在等待（卡在pop的wait里）的worker：被close唤醒，检查队列。如果队列已经空，pop返回false，worker循环退出，线程正常结束。
2.仍然在处理数据的worker：它们已经通过pop拿到Frame，已经退出wait，继续执行图像处理、保存、更新统计的业务逻辑；处理完成后，回到while循环再次调用pop，此时队列已经关闭且为空，pop返回false，worker退出循环。

## 3. 共享统计数据

指出哪些线程会读写 `Statistics`。解释原实现中的竞争为什么可能导致错误结果，并说明
你的同步方案提供了什么保证。还应说明取得快照时为什么是安全的。

producer线程和全部worker线程都会读写Statistics对象，具体读写如下：producer调用onProduced()，worker调用onProcessed()、onSaved()、onCorrupted()，所有线程都可以调用snapshot()读取快照。

原实现中的竞争可能导致错误结果的原因和学长上课讲的一样：原始实现没有互斥锁。i++在计算机里并不是一步完成的，它被拆分为读取数值、自增、写回三步；作业还特意用deliberatelySlowIncrement在读和写之间加入sleep放大概率冲突。多个线程并发执行自增时，多个线程读到同一个旧值，各自+1写回，会出现计数丢失，统计结果偏小。读取快照时，若不加锁，读取快照的四个计数器的过程中其他线程正在修改其中变量，会读到一组不属于同一时刻的不一致脏数据。

我的方案：在Statistics内部增加std::mutex互斥锁。即在onProduced/onProcessed/onSaved/onCorrupted以及snapshot()函数内部，使用std::lock_guard<std::mutex>自动上锁。锁提供互斥保证：同一时刻最多只有一个线程可以进入临界区，不管是修改计数器，还是读取快照。

获取快照时对读取全程加锁，读取期间所有计数更新操作全部暂停。在持有锁期间，其他线程无法进入onProduced等函数修改这几个计数变量，保证四个整数是同一时刻的状态，不会出现读到一部分旧值、一部分新值的情况，保证快照数据一致性，线程安全。

## 4. 线程关闭协议

分别描述以下两条路径中的事件顺序，并解释为什么不会发生 `std::terminate`、悬空访问
或永久等待：

1. 调用者执行 `start()` 后显式调用 `wait()`；
2. 调用者执行 `start()` 后不调用 `wait()`，直接让 `Pipeline` 析构。

如果你的实现允许某个生命周期方法被重复调用，也请说明其行为；如果不允许，请说明前置条件。

1.start()之后显式调用 wait()
`Pipeline::start()`创建producer线程和所有worker线程，线程开始运行。worker进入`workerLoop`，调用`queue_.pop`，大部分worker进入阻塞等待。producer循环读取图像、push帧到队列。

producer读完所有图像，调用`queue_.close()`，唤醒所有等待的worker。

worker处理完剩余队列内的帧，再次pop发现队列关闭且为空，退出workerLoop，worker线程结束。producer完成close 之后,producerLoop执行完毕，producer线程结束。

用户调用`pipeline.wait()`：主线程执行`producer_.join()`等待producer线程结束；循环调用worker的`join()`，等待每一个 worker线程执行完毕。

wait函数全部join完成后返回，此时所有线程都已经结束。不会发生terminate：join会等待线程执行完毕，线程对象joinable状态会变成false，销毁线程对象时不会触发terminate。没有悬空访问：线程退出前所有业务逻辑执行完毕，不再访问Pipeline 成员。不会永久等待：队列close会唤醒所有阻塞worker，所有worker最终都会退出循环。

2.start()之后不调用wait()，直接析构Pipeline
`start()`启动producer与worker线程，流水线正常运行。

Pipeline对象离开作用域，触发析构函数。我们修复后的析构函数内部调用wait()。

wait内部执行join逻辑：阻塞主线程，等待producer线程和所有worker线程全部执行完成。

所有线程join成功，全部线程已经终止，析构继续执行，销毁Pipeline内部成员变量。

保证安全的原理如下：因为会在析构内部强制等待所有线程结束，线程销毁前一定已经join，所以不会触发std::terminate；然后线程是在运行期间访问 Pipeline成员，直到join返回之后Pipeline才继续销毁，所以不会出现悬空访问；队列close机制保证worker最终可以退出循环，不会永久阻塞等待。

前置条件：我的start()最多调用一次。多次调用start会重复创建新线程，workers_vector追加更多线程，会造成重复线程、析构等待大量线程，属于非法调用。本实现不支持重复调用start()，start只能在Pipeline对象生命周期内调用一次。

