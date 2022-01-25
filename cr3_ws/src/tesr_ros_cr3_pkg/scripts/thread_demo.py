import thread

def a(w):
    print(w)
def b(w):
    print(w)

thread.start_new_thread(a, ("this is a function",))
thread.start_new_thread(b, ("this is b function",))

while True:
    pass