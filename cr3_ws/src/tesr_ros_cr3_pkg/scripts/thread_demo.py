import thread

def a(w):
    while True:
        print(w)

def b(w):
    while True:
        print(w)

thread.start_new_thread(a, ("this is a function",))
thread.start_new_thread(b, ("this is b function",))

if __name__ == "__main__":
    while True:
        pass
