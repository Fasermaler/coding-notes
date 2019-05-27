# Pwntools

Author: Fasermaler

Contains references for the use of Pwntools.

I'm adapting this guide from various sources:

- [Pwntools Read The Docs](https://python3-pwntools.readthedocs.io)
- [Gallopsled's pwntools tutorial](https://github.com/Gallopsled/pwntools-tutorial)
- Other sources will be be cited when referenced

## Pre-Requisites

- Basic Python
- Basic Bash
- Some understanding as to how computers work
- Some knowledge of assembly if you intend to use `gdb` and `asm`

## Table of Contents <a name="top"></a> 
1. [Introduction](#1)
2. [Installation](#2)
3. [Getting Started](#3)
   3.1	[Commonly used objects and routines](#3.1)
4. [Context](#4)
   4.1	[Basic Usage](#4.1)
   4.2	[Context Settings](#4.2)
5. [Tubes](#5)
   5.1	[Basic I/O](#5.1)
   ​	5.1.1	[Receiving Data](#5.1.1)
   ​	5.1.2	[Sending Data](#5.1.2)
   ​	5.1.3	[Integer Manipulation](#5.1.3)
   5.2	[Processes and Basic Features](#5.2)
   5.3	[Interacting with Shell](#5.3)
   ​	5.3.1	[Sending commands to Shell](5.3.1)
   5.4	[Networking](#5.4)
   ​	5.4.1	[Connecting using Remote](#5.4.1)
   ​	5.4.2	[Connecting using Socket](#5.4.2)
   ​	5.4.4	[Listening to a client](#5.4.4)
6. [Secure Shell (SSH)](#6)
   6.1	[Multiple SSH Processes](#6.1)
7. [Serial Port Connection](#7)
8. [GNU Project Debugger (GDB)](#8)
   8.1	[Attaching and Interacting with a process](#8.1)
   8.2	[GDB Commands Explained](#8.2)
9. [Assembly](#9)
   9.1	[Command-line Tools](#9.1)
   ​	9.1.1	[Jumping into GDB](#9.1.1)
   9.2	[Basic Assembly](#9.2)
   9.3	[Canned Assembly](#9.3)
10. [Executable and Linkable Format (ELF)](#10)
    10.1	[Loading ELFs](#10.1)
    10.2	[ELF Symbols](#10.2)
    10.3	[Changing Base Addresses](10.3)
    10.4	[Reading ELF Files](#10.4)
    10.5	[Patching ELF Files](#10.5)
    10.6	[Searching within an ELF File](#10.6)
    10.7	[Building ELF Files from scratch](#10.7)

## 1. Introduction <a name="1"></a>

Pwntools is a powerful python library that is useful in writing exploits and CTF games. Pwntools is all about saving time and should be used as such - it is not a penetrative testing suite nor a "cracking script". It merely improves the ease of using existing tools (such as SSH, GDB, ASM). It is also great for learning how computers works on a more fundamental level - as the saying goes, "Hack to learn, not learn to hack"!

Notable classes can be found in the "Getting Started" section while the sections after details specific topics.

Interesting uses I've found for Pwntools include:

- Learning how bash commands work on an assembly level
- De-constructing my own programs to learn about them and find flaws
- Starting ROS processes on a remote computer via SSH (yes)

## 2. Installation <a name="2"></a>
[go to top](#top)

I would recommend working on Pwntools on Linux as it makes things a lot simpler.

Pwntools is available as a `pip` package and can be installed as such:

```shell
$ apt-get update
$ apt-get install python2.7 python-pip python-dev git libssl-dev libffi-dev build-essential
$ pip install --upgrade pip
$ pip install --upgrade pwntools
```

## 3. Getting Started <a name="3"></a>
[go to top](#top)

The easiest way to get started with pwntools is to import everything:

```python
from pwn import *
```

Whichever way Pwntools is imported, the following functions are imported automatically (they are too ubiquitous!):

- `os`
- `sys`
- `time`
- `requests`
- `re`
- `random`

### 3.1 Commonly used objects and routines <a name="3.1"></a>
[go to top](#top)

The following is a list of commonly used objects and routines that are imported (in order of relative use and importance)

- [`pwnlibs.context`](#4)
  -  `pwnlibs.context.context`
  - Used for most of pwntools convenience settings
  - Enable the debug flag using: `context.log_level = 'debug'`
  - It is also possible to disable logging for a subsection of code using `ContextType.local()`
- `remote`, `listen`, `ssh`, `process`
  - `pwnlibs.tubes`
  - Very convenient wrappers that will almost always be used in CTFs - used for connecting to servers or anything really
  - Contains helpers for common tasks like `recvline`, `recvuntil`, `clean`, etc
  - `.interactive()` method allows program to interact directly with the application
- `p32` and `u32`
  - `pwnlib.util.packing` 
  - Functions that let you forget if `'>'` means signed or unsigned for `struct.pack` and removes the need for the `[0]` index at the end
  - Set `signed` and `endian` in a proper manner
  - Common pre-defined sizes (`u8`, `u64`) though you can define your own using `pwnlib.util.packing.pack()`
- `log`
  - `pwnlin.log`
  - Useful for customizing outputs
- `cyclic` and `cyclic_func`
  - ` pwnlib.util.cyclic`
  - Utilities for generating strings such that you can find the offset of 
    any given substring given only N (usually 4) bytes.  This is super 
    useful for straight buffer overflows.  Instead of looking at 0x41414141,
    you could know that 0x61616171 means you control EIP at offset 64 in 
    your buffer.
- `asm` and `disasm`
  - `pwnlib.asm`
  - Awesome module that allows you to turn assembly into bytes or vice versa 
  - Supports any architecture where you have installed binutils
- `shellcraft`
  - `pwnlib.shellcraft`
  - Library of pre-made shellcode
  - `asm(shellcraft.sh())` gives you a shell
- `ELF`
  - `pwnlib.elf`
  - ELF binary manipulation tools, symbol lookup, virtual memory to file offset helpers, abilty to modify and save binaries back to disk
- `DynELF`
  - `pwnlib.dynelf`
  - Dynamically resolve functions given only a pointer to any loaded module and a function which can leak data at any address
- `ROP`
  - `pwnlib.rop`
  - Automatically generate ROP chains using a DSL to describe what you want to do, rather than raw addresses
- `gdb.debug` and `gdb.attach`
  - `pwnlib.gdb`
  - Excellent library that allows you to launch a binay under GDB and pops up a new terminal to interact. Automates setting breakpoints and generally makes using GDB on large applications (or huge iterations) much faster
  - Able to attach to a running process given it's PID using the `pwnlib.tubes` object or even just using a socket connected to it.
- `args`
  - Dictionary containing all-caps command-line args for quick access
- `randoms`, `rol`, `ror`, `xor`, `bits`
  - `pwnlib.util.fiddling`
  - Random operations, allowing for generating random data or simplifying math that requires masking with 0xffffffff or calling `ord` and `chr` a huge number of times
- `net`
  - `pwnlib.util.net`
  - Routines for querying about network interfaces
- `proc`
  - `pwnlib.util.proc`
  - Routines for querying about processes
- `pause`
  - guess what it does
- `safeeval`
  - `pwnlib.util.safeeval`
  - Basically safeguards against evaluating python code



## 4. Context <a name="4">
[go to top](#top)

Before going into anything else, it is important to go over `context` as it is a global, thread-aware object that sets overarching (no pun intended) settings.

**Useful Links**

- [Computer architecture basics](https://www.oreilly.com/library/view/designing-embedded-hardware/0596007558/ch01.html)
- [Comparison of Computer Architectures](https://en.wikipedia.org/wiki/Comparison_of_instruction_set_architectures)
- [Endian Byte Order Explained](https://betterexplained.com/articles/understanding-big-and-little-endian-byte-order/)

### 4.1 Basic Usage <a name="4.1">
[go to top](#top)

The basic context involves setting the architecture for the script/exploit. 

```python
>>>from pwn import *
>>>context.arch = 'amd64' # Sets the architecture to amd64
```

This means that Pwntools will generate shellcode for `amd64` and the default word size will be 64 bits.

It is also possible to clear the context and update it:

```python
>>>context.clear()
>>>context.update(arch='i386', os='linux')
```

This clears the context and updates both the architecture and operating system.

It is also possible to use the `with` keyword to make certain parts of the code context aware.

```python
>>> def nop():
...   print pwnlib.asm.asm('nop').encode('hex')
>>> nop()
00f020e3 
>>> with context.local(arch = 'i386'):
...   nop()
90
```

### 4.2 Context Settings <a name="4.2">
[go to top](#top)

- `arch`

  - Sets the target architecture. Valid values are `"aarch64"`, `"arm"`, `"i386"`, `"amd64"`, etc.  The default is `"i386"`.

    The first time this is set, it automatically sets the default `context.bits` and `context.endian` to the most likely values.

- `bits`

  - Sets the bits to make up a word in the target binary: `32` or `64`

- `binary`

  - Absorb settings from an ELF file

  - Example:

    ```python
    context.binary='/bin/sh'
    ```

- `endian`
  - `big`: Most significant byte is stored first and sent first, followed by the rest in decreasing significance order
  - `little`: Least significant byte is stored first and sent first, followed by the rest in increasing significance order
  - More information on endian [here](https://www.geeksforgeeks.org/little-and-big-endian-mystery/)
- `log_file`
  
  - Defines the logging file to send log output to
- `log_level`
  - Probably one of the most useful features of `context`, it is also scope-aware and can be used with the `with` keyword
  - Possible values: `CRITICAL`, `DEBUG`, `ERROR`, `INFO`, `NOTSET`, `WARN`, `WARNING`
-  `sign`
  
  - Defines integer packings' signed-ness. Default is `unsigned`
- `terminal`
  - Defines the preferred terminal for new windows
  - As a terminal user, I like to set this to `context.terminal = ["terminator", "-e"]`. This allows new windows to be opened in terminator (which is awesome)

## 5. Tubes <a name="5">
[go to top](#top)

Probably some of the most important objects and routines for CTF, Tubes are the I/O wrappers for most types of I/O required:

- Local processes
- Remote [TCP or UDP](http://www.steves-internet-guide.com/tcpip-ports-sockets/) connections
- [SSH](https://www.hostinger.com/tutorials/ssh-tutorial-how-does-ssh-work)
- [Serial port I/O](https://www.codrey.com/embedded-systems/serial-communication-basics/)

(Click on the links if you require explanation for what they are)

For CTF, the most important is going to be SSH and remote TCP/UDP connections - after all, being unable to connect to the game server means nothing can be accomplished.

### 5.1 Basic I/O <a name="5.1">
[go to top](#top)

Some basic I/O operations are covered in this section.

#### 5.1.1 Receiving Data <a name="5.1.1">
[go to top](#top)

- `recv(n)` - Receive any number of available bytes
- `recvline()` - Receive data until a newline is encountered
- `recvuntil(delim)` - Receive data until a delimiter is found
- `recvregex(ex)` - Receive data until a regular expression is completed
- `recvrepeat(timeout)` - Receive data until a timeout
- `clean()` - Discard buffered data

#### 5.1.2 Sending Data <a name="5.1.2">
[go to top](#top)

- `send(data)` - Sends data
- `sendline(line)` - Sends data and a newline

#### 5.1.3 Integer Manipulation <a name="5.1.3">
[go to top](#top)

- `pack(int)` - Send a word-size packed integer
- `unpack()` - Receives and unpacks a word-sized integer

### 5.2 Processes and Basic Features <a name="5.2">
[go to top](#top)

To enable a tube to talk to a process, a `process` object has to be created and pointed to the target binary.

```python
>>> from pwn import *
>>> io = process('sh') #Process is the bash terminal in this case
[x] Starting local process '/bin/sh'
[+] Starting local process '/bin/sh': pid 17276
>>> io.sendline('echo Hello, world')
>>> io.recvline()
'Hello, world\n'
```

There are additional options for the processes. The full documentation is available [here](https://pwntools.readthedocs.org/en/latest/tubes/processes.html).

```python
>>> from pwn import *
>>> io = process(['sh', '-c', 'echo $MYENV'], env={'MYENV': 'MYVAL'})
[x] Starting local process '/bin/sh'
[+] Starting local process '/bin/sh': pid 17300
>>> io.recvline()
'MYVAL\n'
```

It is also possible to read binary data.

```python
>>> from pwn import *
>>> io = process(['sh', '-c', 'echo A; sleep 1; echo B; sleep 1; echo C; sleep 1; echo DDD'])
[x] Starting local process '/bin/sh'
[+] Starting local process '/bin/sh': pid 17318
>>> io.recv()
'A\nB\nC\nDDD\n'
>>> io.recvn(4) # Only receive 4 bytes
'A\nB\n'
>>> hex(io.unpack()) # Unpack io bytes and convert it to hex
'0xa420a41'
```

### 5.3 Interacting with Shell <a name="5.3">
[go to top](#top)

The `.interactive()` method allows you to interact with a remote shell on a game server.

```python
>>>from pwn import *
>>>game_io = process('sh')
>>>game_io.interactive()
$ whoami # Interact with the game shell directly
user
```

#### 5.3.1 Sending commands to Shell <a name="5.3.1">
[go to top](#top)

Using `.sendline()` it is possible to send commands directly to the shell.

```python
>>>from pwn import *
>>>p = process('sh')
[x] Starting local process '/bin/sh'
[+] Starting local process '/bin/sh': pid 19972
>>>p.sendline('./binary')
```

Now the target binary (in this case, named `binary`) will be running.

### 5.4 Networking <a name="5.4">
[go to top](#top)

In the same vein, it is possible to define a remote IO connection (network connection).

#### 5.4.1 Connecting using Remote <a name="5.4.1">
[go to top](#top)

```python
>>>from pwn import *
>>> r = remote('google.com', 443, ssl=True)
>>> r.send('GET /\r\n\r\n')
>>> r.recvn(4)
b'HTTP'
```

#### 5.4.2 Connecting using Socket <a name="5.4.2">
[go to top](#top)

```python
>>> import socket
>>> s = socket.socket() # Define the socket object
>>> s.connect(('google.com', 80))
>>> s.send(b'GET /' + b'\r\n' * 2)
9
>>> r = remote.fromsocket(s)
>>> r.recvn(4)
b'HTTP'
```

#### 5.4.3 Specifying Protocol <a name="5.4.3">
[go to top](#top)

It is also possible to specify the protocol when connecting.

```python
>>>from pwn import *
>>>dns  = remote('8.8.8.8', 53, typ='udp')
>>>tcp6 = remote('google.com', 80, fam='ipv6')
```

#### 5.4.4 Listening to a client <a name="5.4.4">
[go to top](#top)

Listening to the client is also rather easy.

```python
>>>from pwn import *
>>>client = listen(8080).wait_for_connection()
```

### 6. Secure Shell (SSH) <a name="6">
[go to top](#top)

SSH is another bread and butter of CTF games. Most servers will require connection via SSH (or netcat). SSH is a very versatile framework as it allows for portforwarding, file upload / download and etc but requires the target machine to enable it.

```python
>>>from pwn import *
>>>session = ssh('user', 'CTFgameserver.org', password='password')
>>>io = session.process('sh', env={"PS1":""}) # Set the environment to prompt string 1
>>> #<insert exploit here>
>>>io.interactive()
$ whoami
user
```

To learn more about the prompt strings `PS1`, `PS2`, `PS3` and `PS4`, look [here](https://www.thegeekstuff.com/2008/09/bash-shell-take-control-of-ps1-ps2-ps3-ps4-and-prompt_command/). Sometimes PS2 might be better if you intend to run long commands.

Personally, this is an extremely useful class because it allows for complete automation of the CTF task, which makes iteration rather painless.

#### 6. 1 Multiple SSH Processes <a name="6.1">
[go to top](#top)

Creating multiple SSH processes is quite simple - it's possible to simply latch onto the same `session` but create multiple `process` objects. In the following example, it is the equivalent of opening multiple terminals in the remote shell:

```python
>>>from pwn import *
>>>session = ssh('user', 'CTFgameserver.org', password='password')
>>>io1 = session.process('sh', env={"PS1":""}) 
>>>io2 = session.process('sh', env={"PS1":""}) 
```

### 7. Serial Port Connection <a name="7">
[go to top](#top)

In the case of local hacking, Tubes also supports serial connection. I won't go into much detail about this as it is unlikely to be used but here is the [full documentation](https://pwntools.readthedocs.org/en/latest/tubes/serial.html).

```python
>>>from pwn import *
>>>io = serialtube('/dev/ttyUSB0', baudrate=115200)
```

Remember that it is possible to check what devices exist using the following command:

```shell
ls /dev/tty*
```



## 8. GNU Project Debugger (GDB) <a name="8">
[go to top](#top)

GDB is a very useful tool for debugging a target binary. In many cases, the source for the target binary would not be made available and thus GDB allows the user to set breakpoints, check variables and develop an exploit (or not). 

Full GDB documentation is available [here](https://www.gnu.org/software/gdb/documentation/).

You can run GDB normally from the command line via:

```shell
gdb ./target_binary
```

However, in my (humble) opinion, the Pwntools GDB library allows for faster iterations through breakpoints. Especially if the binary is particularly huge. This means that more work can be automated and more time is spent developing an exploit! (or not)

Maybe one day I might see it fit to write a proper stand-alone GDB guide.

### 8.1 Attaching and Interacting with a process <a name="8.1">
[go to top](#top)

To attach to an exiting process use the `attach()` routine.

Attach to a PID directly:

```python
>>>from pwn import *
>>>gdb.attach(1111)
```

Attach to youngest "bash" process:

```python
>>>from pwn import *
>>>gdb.attach('bash')
>>>gdb.sendline('whoami') # Interact with the process
```

Attaching to a remote shell:

```python
>>>server = process(['socat', 'tcp-listen:1234,fork,reuseaddr', 'exec:/bin/sh'])
>>>io = remote('localhost', 1234)
>>>gdb.attach(io, '''
break exit
continue
''') # These extra lines are similar to the GDB commands
>>>io.sendline('exit') # Interacting with the new bash process
```

Starting a process on the remote server:

```python
>>>game_shell = ssh('user', 'CTFgameserver.org', password='password'', port=2220)
>>>cat = shell.process(['cat']) # Starts a new shell process
>>>gdb.attach(cat, '''
break exit
continue
''')
>>>cat.close() # Close the cat process object
```

### 8.2 GDB Commands Explained <a name="8.2">
[go to top](#top)

As seen in the previous section, the `gdb.attach()` (and `gdb.debug()`) can encapsulate GDB commands. Some of the basic commands are as follows:

- `break breakpoint`

  - Sets a breakpoint (it could be a function, a line, etc)

  - Example in GDB shell:

    ```shell
    (gdb) break main
    ```

    Set the process to break at the main function

  - Example in pwnlib GDB:

    ```python
    gdb.attach(io, '''
    break main
    ''')
    ```

- `continue n` 

  - Continues past the previous breakpoint

  - You can specify a number 'n' to denote how many times it will skip the breakpoint (useful when the breakpoint is in a loop). If left empty, this will just move past the current breakpoint

  - Example in GDB shell (c works as well):

    ```shell
    (gdb) continue
    (gdb) c
    ```

  - Example in pwnlib GDB:

    ```python
    gdb.attach(io, '''
    continue
    ''')
    ```

    

GDB shell supports assembly and disassembly as well, but in the case of pwntools, it is better to leave that to the `asm` and `disasm` classes.

## 9. Assembly <a name="9">
[go to top](#top)

Probably one of the finer aspects of pwntools, it can perform assembly in most common architectures and then some. As mentioned in the first section, it also comes with canned shellcode that is reusable and customizable.

This section will be updated as I gain more experience in the matter.

[Assembly Explained](https://www.cs.virginia.edu/~evans/cs216/guides/x86.html)

### 9.1 Command-line Tools <a name="9.1">
[go to top](#top)

These are command-line versions of the pwnlibs.asm library. This allows the user to quickly test shellcode or emit ELF files (or whatever else they'd like to do).

- `asm`

  - Basically assembles assembly code

  - If the output is a terminal, it will be hex-encoded:

    ```shell
    $ asm nop
    90
    ```

    (`nop` is an assembly instruction that does nothing)

  - If the output is anything else, it writes raw data:

    ```shell
    $ asm nop | xxd
    0000000: 90
    ```

    (`xxd` makes a hexdump, very useful tool in CTF!)

  - It takes in data on `stdin` if no instructions are provided on the command line:

    ```shell
    $ echo 'push ebx; pop edi' | asm
    535f
    ```

- `disasm` 

  - Opposite of `asm`, incredibly useful when used in conjunction with GDB to pick apart binaries

  - Example:

    ```shell
    $ disasm cd80
       0:    cd 80                    int    0x80
    ```

- `shellcraft`

  - Directly run shellcode in the terminal. It requires full architecture and os definition

  - Example Echo:

    ```shell
    $ shellcraft i386.linux.echo "Hello, world" STDOUT_FILENO
    686f726c64686f2c20776848656c6c6a04586a015b89e16a0c5acd80
    ```

  - You can get help with the specific shellcode with the `-?` argument

    ```shell
    $ shellcraft i386.linux.cat -?
    Opens a file and writes its contents to the specified file descriptor.
    ```

#### 9.1.1 Jumping into GDB <a name="9.1.1">
[go to top](#top)

One more cool thing is that you can jump straight into GDB from the command line using the `--debug` flag.

`shellcraft` example:

```shell
$ shellcraft i386.linux.sh --debug
```

`asm` example:

```shell
$ asm 'mov eax, 1; int 0x80;' --debug
```

### 9.2 Basic Assembly <a name="9.2">
[go to top](#top)

Converting assembly to shell code is relatively straightforward:

```python
>>>from pwn import *
>>>print repr(asm('xor edi, edi'))
'1\xff'
>>>print enhex(asm('xor edi, edi'))
31ff
```

Unsurprisingly, you can't print assembly out of the box, thus the `repr()` or `enhex()` functions are use to get a desired representation.

### 9.3 Canned Assembly <a name="9.3">
[go to top](#top)

The `shellcraft` module contains plenty of pre-canned, customizable assembly. Because `shellcraft` varies from architecture to architecture, the [full documentation](http://docs.pwntools.com/en/stable/shellcraft.html) is significantly more comprehensive and allows you to peruse `shellcraft` for the appropriate architecture.

Shellcode `cat` example for `aarch64.linux`:

```python
>>> write('flag', 'This is the flag\n')
>>> shellcode = shellcraft.cat('flag') + shellcraft.exit(0)
>>> print disasm(asm(shellcode))
   0:   d28d8cce        mov     x14, #0x6c66                    // #27750
   4:   f2acec2e        movk    x14, #0x6761, lsl #16
   8:   f81f0fee        str     x14, [sp, #-16]!
   c:   d29ff380        mov     x0, #0xff9c                     // #65436
  10:   f2bfffe0        movk    x0, #0xffff, lsl #16
  14:   f2dfffe0        movk    x0, #0xffff, lsl #32
  18:   f2ffffe0        movk    x0, #0xffff, lsl #48
  1c:   910003e1        mov     x1, sp
  20:   aa1f03e2        mov     x2, xzr
  24:   aa1f03e3        mov     x3, xzr
  28:   d2800708        mov     x8, #0x38                       // #56
  2c:   d4000001        svc     #0x0
  30:   aa0003e1        mov     x1, x0
  34:   d2800020        mov     x0, #0x1                        // #1
  38:   aa1f03e2        mov     x2, xzr
  3c:   d29fffe3        mov     x3, #0xffff                     // #65535
  40:   f2afffe3        movk    x3, #0x7fff, lsl #16
  44:   d28008e8        mov     x8, #0x47                       // #71
  48:   d4000001        svc     #0x0
  4c:   aa1f03e0        mov     x0, xzr
  50:   d2800ba8        mov     x8, #0x5d                       // #93
  54:   d4000001        svc     #0x0
>>> run_assembly(shellcode).recvline()
'This is the flag\n'
```



## 10. Executable and Linkable Format (ELF) <a name="10">
[go to top](#top)

[ELFs](https://elinux.org/Executable_and_Linkable_Format_(ELF)) are common file format for executable files, object code, shared libraries and core dumps. Because it is not bound to any processor or architecture, it is widely used and thus is worked with for [reverse engineering, forensics (and other) purposes](http://fluxius.handgrep.se/2011/10/20/the-art-of-elf-analysises-and-exploitations/).

More documentation [here](http://docs.pwntools.com/en/stable/elf.html).

One of the main benefits of using ELFs is to ensure exploits are robust, allowing addresses to be obtained dynamically instead of being hard-oded into the exploit.

### 10.1 Loading ELFs <a name="10.1">
[go to top](#top)

To load an ELF, simply initialize the ELF object pointing to the target file:

```python
>>>from pwn import *
>>> e = ELF('/bin/cat')
```

### 10.2 ELF Symbols <a name="10.1">
[go to top](#top)

ELF objects have different sets of symbols, accessible in the form of attributes:

- `ELF.symbols` lists all known symbols, including those below.  Preference is given the PLT entries over GOT entries.
- `ELF.got` only contains GOT entries
- `ELF.plt` only contains PLT entries
- `ELF.functions` only contains functions (requires DWARF symbols)

Example:

```python
>>>from pwn import *
>>>e = ELF('/bin/bash')
[*] '/bin/bash'
    Arch:     amd64-64-little
    RELRO:    Partial RELRO
    Stack:    Canary found
    NX:       NX enabled
    PIE:      No PIE (0x400000)
    FORTIFY:  Enabled
>>>print "%#x -> license" % e.symbols['bash_license']
0x4ba738 -> license
>>>print "%#x -> execve" % e.symbols['execve']
0x41db60 -> execve
>>>print "%#x -> got.execve" % e.got['execve']
0x6f0318 -> got.execve
>>>print "%#x -> plt.execve" % e.plt['execve']
0x41db60 -> plt.execve
>>>print "%#x -> list_all_jobs" % e.functions['list_all_jobs'].address
0x446420 -> list_all_jobs
```

This allows you to get the addresses dynamically, which can then be parsed or used for other functions, keeping exploits robust and agnostic.

### 10.3 Changing Base Addresses <a name="10.3">
[go to top](#top)

To change a base address, simply update the `.address` attribute of the ELF object. The other symbols will also change to reflect this:

```python
>>>from pwn import *
>>>e = ELF('/bin/bash')
[*] '/bin/bash'
    Arch:     amd64-64-little
    RELRO:    Partial RELRO
    Stack:    Canary found
    NX:       NX enabled
    PIE:      No PIE (0x400000)
    FORTIFY:  Enabled
>>> print "%#x -> base address" % e.address
0x400000 -> base address
>>> print "%#x -> entry point" % e.entry
0x420560 -> entry point
>>> print "%#x -> execve" % e.symbols['execve']
0x41dc20 -> execve
>>> e.address = 0x12340000
>>> print "%#x -> base address" % e.address
0x12340000 -> base address
>>> print "%#x -> entry point" % e.entry
0x12360560 -> entry point
>>> print "%#x -> execve" % e.symbols['execve']
0x1235dc20 -> execve
```

### 10.4 Reading ELF Files <a name="10.4">
[go to top](#top)

Reading ELF files is relatively straightforward as well. Using the `read` function as well as the `repr` function to enable a proper print out.

```python
>>>from pwn import *
>>>e = ELF('/bin/bash')
[*] '/bin/bash'
    Arch:     amd64-64-little
    RELRO:    Partial RELRO
    Stack:    Canary found
    NX:       NX enabled
    PIE:      No PIE (0x400000)
    FORTIFY:  Enabled
>>>print repr(e.read(e.address, 4))
'\x7fELF'
>>>p_license = e.symbols['bash_license']
>>>license   = e.unpack(p_license)
>>>print "%#x -> %#x" % (p_license, license)
0x4ba738 -> 0x4ba640
>>>print e.read(license, 14)
License GPLv3+
>>>print e.disasm(e.symbols['main'], 12)
  41eab0:       41 57                   push   r15
  41eab2:       41 56                   push   r14
  41eab4:       41 55                   push   r13
```

### 10.5 Patching ELF Files <a name="10.5">
[go to top](#top)

In this case, we can patch the bash ELF file to obtain a modified version of bash. Do be careful when saving so as to not mess up the original files.

```python
>>>from pwn import *
>>>e = ELF('/bin/bash')

# Cause a debug break on the 'exit' command
>>>e.asm(e.symbols['exit_builtin'], 'int3')

# Disable chdir and just print it out instead
>>>e.pack(e.got['chdir'], e.plt['puts'])

# Change the license
>>>p_license = e.symbols['bash_license']
>>>license = e.unpack(p_license)
>>>e.write(license, 'Hello, world!\n\x00')

>>>e.save('./bash-modified')
```

We can then run `bash-modified` which is saved in our working directory to yield some peculiar behaviors.

```shell
$ chmod +x ./bash-modified
$ ./bash-modified -c 'exit'
Trace/breakpoint trap (core dumped)
$ ./bash-modified --version | grep "Hello"
Hello, world!
$ ./bash-modified -c 'cd "No chdir for you!"'
/home/user/No chdir for you!
No chdir for you!
./bash-modified: line 0: cd: No chdir for you!: No such file or directory
```

### 10.6 Searching within an ELF File <a name="10.6">
[go to top](#top)

The following is the documentation recommended format for searching through an ELF file for a specific `execve` call.

```python
>>>from pwn import *

>>>e = ELF('/bin/bash')

>>>for address in e.search('/bin/sh\x00'):
...    print hex(address)
```

This will return addresses for the specific `execve` call.

### 10.7 Building ELF Files from scratch <a name="10.7">
[go to top](#top)

The following are some simple examples on creating an ELF file from scratch. All of these functions return an ELF object which can be saved.

`from_bytes`:

```python
>>>from pwn import *
>>>e = ELF.from_bytes('\xcc')
>>>e.save('int3-1')
```

`from_assembly`:

```python
>>>from pwn import *
>>>e = ELF.from_assembly('nop', arch='powerpc')
>>>e.save('powerpc-nop')
```



