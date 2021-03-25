from multiprocessing import Process, Manager, Value


def f1(v):
    v = 1


def f2(v):
    while 1:
        if v == 1:
            print("a")


if __name__ == '__main__':
    with Manager() as manager:
        # d = manager.dict()
        # l1 = manager.list(range(10))
        # l2 = manager.list(range(8))
        v = Value('i', 0)

        p1 = Process(target=f1, args=(v,))
        p2 = Process(target=f2, args=(v,))
        p1.start()
        p2.start()
        p1.join()
        p2.join()
