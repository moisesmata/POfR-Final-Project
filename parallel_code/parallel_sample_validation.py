import threading
import math

def validation_worker(self, i, q_start, q_end, step_size, stop_event):
    if stop_event.is_set():
        return
    q_seg = self.step(q_start, q_end, step_size=i * step_size)
    if not self.is_state_valid(q_seg):
        stop_event.set()
    

def is_segment_valid(self, q_start, q_end):

    threads = []
    stop_event = threading.Event()

    if self.current_obstacle == 'NONE':
        step_size = 0.05
    elif self.current_obstacle == 'SIMPLE':
        step_size = 0.02
    elif self.current_obstacle == 'HARD':
        step_size = 0.01
    elif self.current_obstacle == 'SUPER':
        step_size = 0.005

    direction = [qr - qn for qr, qn in zip(q_end, q_start)]
    length = math.sqrt(sum([d**2 for d in direction]))
    num_segs = int(length / step_size)
    for i in range(num_segs + 1):
        thread = threading.Thread(target=self.validation_worker, args=(i, q_start, q_end, step_size, stop_event))
        threads.append(thread)
        thread.start()
        if stop_event.is_set():
            return False

    for thread in threads:
        thread.join()
    return not stop_event.is_set()
