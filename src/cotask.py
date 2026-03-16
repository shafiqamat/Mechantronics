"""Cooperative task scheduling for MicroPython multitasking.

This module provides classes to run cooperatively scheduled tasks in a
multitasking system. Tasks are created as generators: functions with infinite
loops that call ``yield`` at least once per loop. References to all
tasks are kept in a list maintained by :class:`TaskList`; the scheduler runs
each task's generator according to a chosen algorithm (e.g. round-robin or
highest-priority-first).

Example:
    Create a task and run it with the scheduler::

        def task1_fun():
            '''Switch states repeatedly.'''
            state = 0
            while True:
                if state == 0:
                    state = 1
                elif state == 1:
                    state = 0
                yield state

        task1 = cotask.Task(task1_fun, name='Task 1', priority=1,
                            period=500, profile=True, trace=True)
        cotask.task_list.append(task1)
        while True:
            cotask.task_list.pri_sched()

Module contents:
    - **Task** – Cooperative task with optional profiling and tracing.
    - **TaskList** – Container of tasks; supports round-robin and
      priority scheduling.
    - **SCHEDULER_DEBUG_MAX** – Maximum number of scheduler debug entries
      to keep (default 64; oldest dropped when full).
    - **task_list** – Default :class:`TaskList` instance created on import;
      append tasks here and call :meth:`TaskList.pri_sched` or
      :meth:`TaskList.rr_sched` to run them.

Copyright (c) 2017-2023 JR Ridgely. Released under the GNU Public License v3.0.
Intended for educational use; use is not limited thereto.
"""

import gc                              # Memory allocation garbage collector
import utime                           # Micropython version of time library
import micropython                     # This shuts up incorrect warnings


class Task:
    """Cooperative task with scheduling and optional performance logging.

    Implements behavior common to tasks in a cooperative multitasking system
    under MicroPython. A task can be scheduled by time (period) or by an
    external trigger (e.g. interrupt or another task). State transitions can
    be traced and run times profiled. Task code must be a generator that
    yields the current state (and the CPU) after running for a short, bounded
    time.
    """

    def __init__(self, run_fun, name="NoName", priority=0, period=None,
                 profile=False, trace=False, shares=()):
        """Initialize a task so it may be run by the scheduler.

        Args:
            run_fun: Generator function that implements the task. Must yield
                the current state.
            name: Short descriptive name for the task. Default is "NoName".
            priority: Positive integer; higher means higher priority.
                Default 0.
            period: Time in milliseconds between runs when using timer
                scheduling, or None if the task is triggered only by go().
                Accepts int or float; converted to microseconds internally.
            profile: If True, enable run-time profiling (execution time stats).
            trace: If True, record state transition list. Note: uses memory
                and can slow execution.
            shares: Optional list or tuple of shares/queues used by this task.
                If omitted, no shares are passed to the task.
        """
        # The function which is run to implement this task's code. Since it
        # is a generator, we "run" it here, which doesn't actually run it but
        # gets it going as a generator which is ready to yield values
        if shares:
            self._run_gen = run_fun(shares)
        else:
            self._run_gen = run_fun()

        ## The name of the task, hopefully a short and descriptive string.
        self.name = name

        ## The task's priority, an integer with higher numbers meaning higher 
        #  priority. 
        self.priority = int(priority)

        ## The period, in milliseconds, between runs of the task's @c run()
        #  method. If the period is @c None, the @c run() method won't be run
        #  on a time basis but will instead be run by the scheduler as soon
        #  as feasible after code such as an interrupt handler calls the 
        #  @c go() method. 
        if period != None:
            self.period = int(period * 1000)
            self._next_run = utime.ticks_us() + self.period
        else:
            self.period = period
            self._next_run = None

        # Flag which causes the task to be profiled, in which the execution
        #  time of the @c run() method is measured and basic statistics kept. 
        self._prof = profile
        self.reset_profile()

        # The previous state in which the task last ran. It is used to watch
        # for and track state transitions.
        self._prev_state = 0

        # If transition tracing has been enabled, create an empty list in 
        # which to store transition (time, to-state) stamps
        self._trace = trace
        self._tr_data = []
        self._prev_time = utime.ticks_us()

        ## Flag which is set true when the task is ready to be run by the
        #  scheduler
        self.go_flag = False


    def schedule(self) -> bool:
        """Run this task once if it is ready (called by the scheduler).

        If the task is not ready, returns immediately. If ready, runs the
        task's generator up to the next yield() and then returns.

        Returns:
            True if the task ran, False if it did not.
        """
        if self.ready():

            # Reset the go flag for the next run
            self.go_flag = False

            # If profiling, save the start time
            if self._prof:
                stime = utime.ticks_us()

            # Run the method belonging to the state which should be run next
            curr_state = next(self._run_gen)

            # If profiling or tracing, save timing data
            if self._prof or self._trace:
                etime = utime.ticks_us()

            # If profiling, save timing data
            if self._prof:
                self._runs += 1
                runt = utime.ticks_diff(etime, stime)
                if self._runs > 2:
                    self._run_sum += runt
                    if runt > self._slowest:
                        self._slowest = runt

            # If transition logic tracing is on, record a transition; if not,
            # ignore the state. If out of memory, switch tracing off and 
            # run the memory allocation garbage collector
            if self._trace:
                try:
                    if curr_state != self._prev_state:
                        self._tr_data.append(
                            (utime.ticks_diff(etime, self._prev_time),
                             curr_state))
                except MemoryError:
                    self._trace = False
                    gc.collect()

                self._prev_state = curr_state
                self._prev_time = etime

            return True

        else:
            return False


    @micropython.native
    def ready(self) -> bool:
        """Return whether the task is ready to run.

        For timer-based tasks, checks if the next run time has been reached.
        For event-triggered tasks, checks the go flag. Can be overridden in
        subclasses for custom readiness logic.

        Returns:
            True if the task is ready to run, False otherwise.
        """
        # If this task uses a timer, check if it's time to run run() again. If
        # so, set go flag and set the timer to go off at the next run time
        if self.period != None:
            late = utime.ticks_diff(utime.ticks_us(), self._next_run)
            if late > 0:
                self.go_flag = True
                self._next_run = utime.ticks_diff(self.period, 
                                                  -self._next_run)

                # If keeping a latency profile, record the data
                if self._prof:
                    self._late_sum += late
                    if late > self._latest:
                        self._latest = late

        # If the task doesn't use a timer, we rely on go_flag to signal ready
        return self.go_flag


    def set_period(self, new_period):
        """Set the period between task runs.

        Args:
            new_period: New period in milliseconds, or None to make the task
                event-triggered (via go()) instead of time-based.
        """
        if new_period is None:
            self.period = None
        else:
            self.period = int(new_period) * 1000


    ## This method resets the variables used for execution time profiling.
    #  This method is also used by @c __init__() to create the variables.
    def reset_profile(self):
        """Reset execution-time profiling counters.

        Also used internally by __init__() to initialize profiling variables.
        """
        self._runs = 0
        self._run_sum = 0
        self._slowest = 0
        self._late_sum = 0
        self._latest = 0


    def get_trace(self):
        """Return a string with the task's state transition trace.

        The trace lists (time, from_state, to_state) for each transition,
        if tracing was enabled. Can be large.

        Returns:
            String showing state transitions, or "not traced" if tracing
            was disabled.
        """
        tr_str = 'Task ' + self.name + ':'
        if self._trace:
            tr_str += '\n'
            last_state = 0
            total_time = 0.0
            for item in self._tr_data:
                total_time += item[0] / 1000000.0
                tr_str += '{: 12.6f}: {: 2d} -> {:d}\n'.format (total_time, 
                    last_state, item[1])
                last_state = item[1]
        else:
            tr_str += ' not traced'
        return tr_str


    def go(self):
        """Mark the task as ready to run.

        Can be called from an interrupt service routine or from another task
        when data is available for this task to process.
        """
        self.go_flag = True


    def __repr__(self):
        """String representation for diagnostics.

        Shows task name, priority, period, run count, and (if profiled) avg/max
        execution time and latency. Column widths are kept short (~60 chars)
        for narrow serial consoles.

        Returns:
            Single-line string representing the task.
        """
        # Keep name to 16 chars so full line fits in ~62 chars (no wrap)
        name = self.name[:16] if len(self.name) > 16 else self.name
        rst = f"{name:<16s}{self.priority:>3d}"
        try:
            rst += f"{(self.period / 1000.0):>7.1f}"
        except TypeError:
            rst += '      -'
        rst += f"{self._runs:>6d}"

        if self._prof and self._runs > 0:
            avg_dur = (self._run_sum / self._runs) / 1000.0
            avg_late = (self._late_sum / self._runs) / 1000.0
            rst += f"{avg_dur:>7.2f}   {(self._slowest / 1000.0):>6.2f} "
            if self.period != None:
                rst += f"{avg_late:>7.2f}    {(self._latest / 1000.0):>6.2f}"
        return rst


# =============================================================================

# Maximum number of scheduler debug entries to keep (oldest dropped when full).
SCHEDULER_DEBUG_MAX = 64


class TaskList:
    """List of tasks used by the scheduler.

    Holds all tasks that will be run by the scheduler. Typically you only
    add tasks (append) and call the scheduler (rr_sched or pri_sched). The
    list is sorted by priority so the scheduler can quickly find the
    highest-priority ready task. Scheduling can be priority-based or
    round-robin.
    """

    def __init__(self):
        """Initialize the task list.

        Creates an empty priority list structure. pri_list holds sublists
        per priority level; each sublist has [priority, index, task, ...].
        """
        self.pri_list = []
        self.scheduler_debug=[]


    ## Append a task to the task list. The list will be sorted by task 
    #  priorities so that the scheduler can quickly find the highest priority
    #  task which is ready to run at any given time. 
    #  @param task The task to be appended to the list
    def append(self, task):
        """Append a task to the list.

        The list is kept sorted by priority (highest first) so the scheduler
        can quickly find the highest-priority ready task.

        Args:
            task: Task instance to add (e.g. a :class:`Task`).
        """
        # See if there's a tasklist with the given priority in the main list
        new_pri = task.priority
        for pri in self.pri_list:
            # If a tasklist with this priority exists, add this task to it.
            if pri[0] == new_pri:
                pri.append(task)
                break

        # If the priority isn't in the list, this else clause starts a new 
        # priority list with this task as first one. A priority list has the
        # priority as element 0, an index into the list of tasks (used for
        # round-robin scheduling those tasks) as the second item, and tasks
        # after those
        else:
            self.pri_list.append([new_pri, 2, task])

        # Make sure the main list (of lists at each priority) is sorted
        self.pri_list.sort(key=lambda pri: pri[0], reverse=True)


    @micropython.native
    def rr_sched(self):
        """Run tasks in round-robin order, ignoring priority.

        Each call iterates through all tasks and gives each one a chance to
        run. Higher-priority tasks are visited first, but over time every task
        runs about equally often.
        """
        # For each priority level, run all tasks at that level
        for pri in self.pri_list:
            for task in pri[2:]:
                task.schedule()


    @micropython.native
    def pri_sched(self):
        """Run the single highest-priority task that is ready.

        Each call finds the highest-priority task that is ready and runs it
        once (one generator step). Within a priority level, tasks are
        scheduled in round-robin order.
        """
        # Go down the list of priorities, beginning with the highest
        for pri in self.pri_list:
            # Within each priority list, run tasks in round-robin order
            # Each priority list is [priority, index, task, task, ...] where
            # index is the index of the next task in the list to be run
            tries = 2
            length = len(pri)
            while tries < length:
                dstart = utime.ticks_us()
                ran = pri[pri[1]].schedule()
                scheduler_time = utime.ticks_diff(utime.ticks_us(), dstart)
                # Keep only the last SCHEDULER_DEBUG_MAX entries
                if len(self.scheduler_debug) >= SCHEDULER_DEBUG_MAX:
                    self.scheduler_debug.pop(0)
                self.scheduler_debug.append([pri[pri[1]], f'{scheduler_time:.3f}'])
                tries += 1
                pri[1] += 1
                if pri[1] >= length:
                    pri[1] = 2
                if ran:
                    return


    def __repr__(self):
        ret_str = '\r\nTASK              PRI PERIOD RUNS  AVG_MS   MAX_MS AVG_LAT  MAX_LAT\r\n'
        for pri in self.pri_list:
            for task in pri[2:]:
                ret_str += str(task) + '\r\n'

        return ret_str


# Main task list created when cotask is imported; add tasks here and call
# rr_sched() or pri_sched() to run them.
task_list = TaskList()




