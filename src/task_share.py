"""Thread-safe queues and shared variables for cooperative tasks.

This module provides queue and share abstractions that allow tasks to pass data
without risk of corruption from interrupts or pre-emptive multithreading.
It is designed for MicroPython-based embedded systems using the ``pyb`` API.

Typical usage:

    Using a queue::

        import task_share

        # This queue holds unsigned short (16-bit) integers
        my_queue = task_share.Queue('H', 100, name='My Queue')

        # In one task, put data into the queue
        my_queue.put(some_data)

        # In another task, read data from the queue
        something = my_queue.get()

    Using a share::

        import task_share

        # This share holds a signed short (16-bit) integer
        my_share = task_share.Share('h', name='My Share')

        # In one task, write data
        my_share.put(some_data)

        # In another task, read data
        something = my_share.get()

Module contents:
    - **BaseShare** – Common base class for :class:`Queue` and :class:`Share`.
    - **Queue** – Ring buffer for passing multiple values between tasks.
    - **Share** – Single-value container for sharing the latest value.
    - **share_list** – Global list of all shares and queues (for diagnostics).
    - **type_code_strings** – Mapping from ``array`` type codes to readable
      type names.
    - **show_all()** – Return a diagnostic string describing all shares/queues.

Copyright (c) 2017-2023 JR Ridgely.
Released under the GNU Public License v3.0.
"""

import array
import gc
import pyb
import micropython


## This is a system-wide list of all the queues and shared variables. It is
#  used to create diagnostic printouts. 
share_list = []

## This dictionary allows readable printouts of queue and share data types.
type_code_strings = {'b' : "int8",   'B' : "uint8",
                     'h' : "int16",  'H' : "uint16",
                     'i' : "int(?)", 'I' : "uint(?)",
                     'l' : "int32",  'L' : "uint32",
                     'q' : "int64",  'Q' : "uint64",
                     'f' : "float",  'd' : "double"}


def show_all():
    """Return a diagnostic string describing all queues and shares.

    Returns:
        str: One line per share/queue, joined with newlines.
    """
    gen = (str (item) for item in share_list)
    return '\n'.join (gen)


class BaseShare:
    """Base class for queues and shares which exchange data between tasks.

    This class isn't used directly; it just factors out behavior common to
    :class:`Queue` and :class:`Share` (type code, thread protection, and
    registration in :data:`share_list`).
    """

    def __init__ (self, type_code, thread_protect=True, name=None):
        """Initialize the base part of a queue or share.

        Args:
            type_code: One-letter type code as used by :class:`array.array`.
            thread_protect: If True, disable/enable IRQs around accesses so
                data is safe in the presence of interrupts or pre-emption.
            name: Optional human-readable name for diagnostics.
        """
        self._type_code = type_code
        self._thread_protect = thread_protect

        # Add this queue to the global share and queue list
        share_list.append (self)


class Queue (BaseShare):
    """Queue used to transfer data from one task to another.

    A :class:`Queue` is a ring buffer holding elements of a uniform type
    (given by an ``array`` type code). It is safe for use with interrupts or
    pre-emptive scheduling if ``thread_protect`` is enabled.

    Example::

        import task_share

        # Unsigned 16-bit samples
        samples = task_share.Queue('H', 100, name='Samples')

        # In producer task
        samples.put(value)

        # In consumer task
        if samples.any():
            v = samples.get()
    """

    ser_num = 0

    def __init__ (self, type_code, size, thread_protect=False,
                  overwrite=False, name=None):
        """Initialize a queue for passing data between tasks.

        Each queue carries items of a single type. The type is specified by
        an ``array`` type code, one of:

        - ``b`` / ``B`` – 8-bit (signed/unsigned)
        - ``h`` / ``H`` – 16-bit
        - ``i`` / ``I`` – native-width ints
        - ``l`` / ``L`` – 32-bit
        - ``q`` / ``Q`` – 64-bit
        - ``f`` / ``d`` – (double-precision) float

        Args:
            type_code: One-letter type code for the data items.
            size: Maximum number of items the queue can hold.
            thread_protect: If True, disable/enable IRQs around accesses.
            overwrite: If True, overwrite oldest data when queue is full.
            name: Optional name; defaults to ``QueueN`` where ``N`` is a
                serial number.
        """
        # First call the parent class initializer
        super ().__init__ (type_code, thread_protect, name)

        self._size = size
        self._overwrite = overwrite
        self._name = str (name) if name != None \
            else 'Queue' + str (Queue.ser_num)
        Queue.ser_num += 1

        # Allocate memory in which the queue's data will be stored
        try:
            self._buffer = array.array (type_code, range (size))
        except MemoryError:
            self._buffer = None
            raise
        except ValueError:
            self._buffer = None
            raise

        # Initialize pointers to be used for reading and writing data
        self.clear ()

        # Since we may have allocated a bunch of memory, call the garbage
        # collector to neaten up what memory is left for future use
        gc.collect ()


    @micropython.native
    def put (self, item, in_ISR = False):
        """Put an item into the queue.

        If the queue is full and ``overwrite`` is False, this method blocks
        (busy-waits) until there is room. In an ISR, it returns immediately
        if the queue is full.

        Args:
            item: Value to enqueue.
            in_ISR: True if called from an interrupt context.
        """
        # If we're in an ISR and the queue is full and we're not allowed to
        # overwrite data, we have to give up and exit
        if self.full ():
            if in_ISR:
                return

            # Wait (if needed) until there's room in the buffer for the data
            if not self._overwrite:
                while self.full ():
                    pass

        # Prevent data corruption by blocking interrupts during data transfer
        if self._thread_protect and not in_ISR:
            _irq_state = pyb.disable_irq ()

        # Write the data and advance the counts and pointers
        self._buffer[self._wr_idx] = item
        self._wr_idx += 1
        if self._wr_idx >= self._size:
            self._wr_idx = 0
        self._num_items += 1
        if self._num_items >= self._size:        # Can't be fuller than full
            self._num_items = self._size
        if self._num_items > self._max_full:     # Record maximum fillage
            self._max_full = self._num_items

        # Re-enable interrupts
        if self._thread_protect and not in_ISR:
            pyb.enable_irq (_irq_state)


    @micropython.native
    def get (self, in_ISR = False):
        """Read and remove one item from the queue.

        This method blocks (busy-waits) until an item is available.

        Args:
            in_ISR: True if called from interrupt context.

        Returns:
            The dequeued item.
        """
        # Wait until there's something in the queue to be returned
        while self.empty ():
            pass

        # Prevent data corruption by blocking interrupts during data transfer
        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq ()

        # Get the item to be returned from the queue
        to_return = self._buffer[self._rd_idx]

        # Move the read pointer and adjust the number of items in the queue
        self._rd_idx += 1
        if self._rd_idx >= self._size:
            self._rd_idx = 0
        self._num_items -= 1
        if self._num_items < 0:
            self._num_items = 0

        # Re-enable interrupts
        if self._thread_protect and not in_ISR:
            pyb.enable_irq (irq_state)

        return (to_return)


    @micropython.native
    def any (self):
        """Return True if there is at least one item in the queue."""
        return (self._num_items > 0)


    @micropython.native
    def empty (self):
        """Return True if the queue is empty."""
        return (self._num_items <= 0)


    @micropython.native
    def full (self):
        """Return True if the queue is full (no more room for new items)."""
        return (self._num_items >= self._size)


    @micropython.native
    def num_in (self):
        """Return the current number of items in the queue."""
        return (self._num_items)


    def clear (self):
        """Remove all contents from the queue and reset counters."""
        self._rd_idx = 0
        self._wr_idx = 0
        self._num_items = 0
        self._max_full = 0


    def __repr__ (self):
        """Return a diagnostic string describing this queue."""
        return ('{:<12s} Queue<{:s}> Max Full {:d}/{:d}'.format (self._name,
                type_code_strings[self._type_code], self._max_full, self._size))


# ============================================================================

class Share (BaseShare):
    """Single-value shared data item used between tasks.

    A :class:`Share` holds the latest value of a single variable of a given
    type. With ``thread_protect`` enabled, reads and writes are protected
    against corruption by interrupts or pre-emptive scheduling.

    Example::

        import task_share

        my_share = task_share.Share('h', name='My Share')
        my_share.put(value)
        latest = my_share.get()
    """

    ## A counter used to give serial numbers to shares for diagnostic use.
    ser_num = 0


    ## Create a shared data item used to transfer data between tasks.
    # 
    #  This method allocates memory in which the shared data will be buffered.
    # 
    #  Each share can only carry data of one particular type which must be
    #  chosen from the following list. The data type is specified by a 
    #  one-letter type code which is given as for the Python @c array.array
    #  type, which can be any of the following:
    #  |      |      |      |
    #  |:-----|:-----|:-----|
    #  | **b** (signed char) | **B** (unsigned char) | 8 bit integers |
    #  | **h** (signed short) | **H** (unsigned short) | 16 bit integers |
    #  | **i** (signed int) | **I** (unsigned int) | 32 bit integers (probably) |
    #  | **l** (signed long) | **L** (unsigned long) | 32 bit integers |
    #  | **q** (signed long long) | **Q** (unsigned long long) | 64 bit integers |
    #  | **f** (float) | **d** (double-precision float) | |
    # 
    #  @param type_code The type of data items which the share can hold
    #  @param thread_protect True if mutual exclusion protection is used
    #  @param name A short name for the share, default @c ShareN where @c N
    #         is a serial number for the share
    def __init__ (self, type_code, thread_protect = True, name = None):
        # First call the parent class initializer
        super ().__init__ (type_code, thread_protect, name)

        self._buffer = array.array (type_code, [0])

        self._name = str (name) if name != None \
            else 'Share' + str (Share.ser_num)
        Share.ser_num += 1


    ## Write an item of data into the share.
    # 
    #  This method puts data into the share; any old data is overwritten.
    #  This code disables interrupts during the writing so as to prevent
    #  data corrupting by an interrupt service routine which might access
    #  the same data.
    #  @param data The data to be put into this share
    #  @param in_ISR Set this to True if calling from within an ISR
    @micropython.native
    def put (self, data, in_ISR = False):
        """Write a new value into the share, overwriting any previous value.

        Args:
            data: New value to store.
            in_ISR: True if called from interrupt context.
        """

        # Disable interrupts before writing the data
        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq ()

        self._buffer[0] = data

        # Re-enable interrupts
        if self._thread_protect and not in_ISR:
            pyb.enable_irq (irq_state)


    @micropython.native
    def get (self, in_ISR = False):
        """Read and return the current value of the share.

        Args:
            in_ISR: True if called from interrupt context.

        Returns:
            The value currently stored in the share.
        """
        # Disable interrupts before reading the data
        if self._thread_protect and not in_ISR:
            irq_state = pyb.disable_irq ()

        to_return = self._buffer[0]

        # Re-enable interrupts
        if self._thread_protect and not in_ISR:
            pyb.enable_irq (irq_state)

        return (to_return)


    ## Puts diagnostic information about the share into a string.
    #
    #  Shares are pretty simple, so we just put the name and type. 
    def __repr__ (self):
        """Return a simple representation of the share (current value)."""
        return str (self.get ())  # return the current value of the share

