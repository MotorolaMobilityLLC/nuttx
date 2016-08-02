#
# Copyright (c) 2016 Motorola Mobility, LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# GDB commands to ease the debugging of nuttx.
#
# Load in gdb with:
#   source -s nuttx/tools/gdb_nuttx_data.py
#
# Notes:
#   1. Whenever possible memory should be read in blocks.  Reading one
#      entry at a time can be quite slow.  See the Dump* functions for examples.
#
import gdb
import operator
from collections import OrderedDict
from curses.ascii import isprint
from struct import unpack_from, pack_into

ARM_THUMB_MASK = 0xfffffffe

def ReadWriteMemory(address, size):
    address = int(address.cast(gdb.lookup_type('unsigned int')))
    inferior = gdb.selected_inferior()
    try:
        mem = inferior.read_memory(address, size);
    except:
        raise gdb.GdbError("Unable to read memory at 0x{:08x}".format(address))
    try:
        inferior.write_memory(address, buf)
    except:
        raise gdb.GdbError("Unable to write the memory at 0x{:08x}".format(address))

def ReadMemory(address, size):
    inferior = gdb.selected_inferior()
    try:
        mem = inferior.read_memory(address, size);
    except:
        raise gdb.GdbError("Unable to read memory at 0x{:08x}".format(address))

    return memoryview(mem)

def ReadMemoryCast(address, size):
    mem_p = address.cast(gdb.lookup_type('void').pointer())
    return ReadMemory(mem_p, size)

def WriteMemory(address, buf, size):
    inferior = gdb.selected_inferior()
    #print ''.join('{:02x}'.format(ord(x)) for x in buf)
    try:
        inferior.write_memory(address, buf)
    except:
        raise gdb.GdbError("Unable to write the memory at 0x{:08x}".format(int(address.cast(gdb.lookup_type('unsigned int')))))

def WriteMemoryCast(address, buf, size):
    mem_p = address.cast(gdb.lookup_type('void').pointer())
    return WriteMemory(mem_p, str(buf), size)

def DumpHex(str, fmt, address, size):
    out_fmt = {
        'B' : '{:02x} ',
        'H' : '{:04x} ',
        'I' : '{:08x} ',
        'Q' : '{:16x} '
    }
    out_unknown = {
        'B' : 'xx ',
        'H' : 'xxxx ',
        'I' : 'xxxxxxxx ',
        'Q' : 'xxxxxxxxxxxxxxxx '
    }
    out_size = {
        'B' : 1,
        'H' : 2,
        'I' : 4,
        'Q' : 8
    }

    #Adjust size to have enough for the type of data requested.
    if (size%out_size[fmt] != 0):
       size += out_size[fmt]-(size%out_size[fmt])

    mem = ReadMemory(address, size)
    j = 0
    hex_str = "{:08x}: ".format(address);
    chr_str = ""
    # Output the data read from memory
    while (size > j):
        # Print the data
        if (j%out_size[fmt] == 0):
            hex_str += out_fmt[fmt].format(unpack_from(fmt, mem, j)[0])
        byte = chr(unpack_from("B", mem, j)[0])
        if (isprint(byte)):
            chr_str += byte
        else:
            chr_str += '.'
        j += 1
        if (j%16 == 0):
            print str + hex_str + chr_str
            hex_str = "{:08x}: ".format(address+j);
            chr_str = ""
    #Take care of any trailing data.
    if (j%16 != 0):
        while (j%16 != 0):
            if (j%out_size[fmt] == 0):
                hex_str += out_unknown[fmt]
            chr_str += "."
            j += 1;
        print str + hex_str + chr_str

def DumpHexArg(fmt, arg):
    argv = gdb.string_to_argv(arg)
    if (len(argv) != 2):
        raise gdb.GdbError("dump hex <address|symbol> <bytes>")

    address = gdb.parse_and_eval(argv[0])
    try:
        size = int(gdb.parse_and_eval(argv[1]))
    except ValueError:
        raise gdb.GdbError("The number of bytes is invalid.")

    address = int(address.cast(gdb.lookup_type("unsigned int")))
    DumpHex("", fmt, address, size)

class DumpSyslogCmd(gdb.Command):
    "Command for dumping the system log (/dev/ramlog) from nuttx."

    def __init__(self):
        super (DumpSyslogCmd, self).__init__ ("dump syslog", gdb.COMMAND_DATA, gdb.COMPLETE_NONE)

    def invoke (self, arg, from_tty):
        g_sysdev = gdb.parse_and_eval("g_sysdev")
        head = int(g_sysdev['rl_head'])
        tail = int(g_sysdev['rl_tail'])
        size = int(g_sysdev['rl_bufsize'])
        mem = ReadMemoryCast(g_sysdev['rl_buffer'], size)

        print "Syslog (Size: {:d} Head: {:d} Tail: {:d}:".format(size, head, tail)
        str = ""
        while (tail != head):
            if (tail >= size):
                tail = 0;
            byte = chr(unpack_from("B", mem, tail)[0])
            if (byte == '\0') or (byte == '\n'):
                print str
                str = ""
            elif (byte != '\r'):
               str += byte
            tail += 1
        print str

class DumpRegLogCmd(gdb.Command):
    "Command for dumping a register log taken with the custom register log code."

    def __init__(self):
        super (DumpRegLogCmd, self).__init__ ("dump reglog", gdb.COMMAND_DATA, gdb.COMPLETE_NONE)

    def invoke (self, arg, from_tty):
        g_reglog = gdb.parse_and_eval("g_reglog")
        tail = int(g_reglog['tail'])
        head = int(g_reglog['head'])
        # Determine the time base.
        timebase = 100;
        clock_fcn = gdb.lookup_symbol('tsb_fr_tmr_get')[0]
        if clock_fcn:
            if ((int(clock_fcn.value().cast(gdb.lookup_type('unsigned int'))) & ARM_THUMB_MASK) ==
                (int(g_reglog['get_time_fcn'].cast(gdb.lookup_type('unsigned int'))) & ARM_THUMB_MASK)):
                timebase = 48000000
        if (head != tail):
            # Get the size of each element in the array.
            record_size = g_reglog['log'][0].type.sizeof
            log_len = g_reglog['log'].type.sizeof/record_size;
            overwritten = 0;
            if (head-tail > log_len):
                overwritten = head-tail-log_len
            print "Head: {:d} Tail: {:d}".format(head, tail)
            print "Values logged: {:d} Overwritten: {:d} Timebase: {:d}".format(head-tail, overwritten, timebase)
            # Find the positions of the structure members.
            bytepos = dict()
            pos = 0
            for field in g_reglog['log'][0].type.fields():
                bytepos[field.name] = pos
                pos += 1
            # Setup for the loop.
            i = tail % log_len
            last = head % log_len
            j = 1
            print "Index Time(sec)    Address     Value"
            mem = ReadMemoryCast(g_reglog['log'].address, log_len*record_size)
            while (last != i):
                # Read in one element of the array.  Please note this
                # assumes 3 elements in the array all of type unsigned int.
                data = unpack_from("III", mem, i*record_size)
                time = int(data[bytepos['time']])
                addr = int(data[bytepos['addr']])
                val = int(data[bytepos['val']])
                print "{:-4d} {:12.9f}  0x{:08x}  0x{:08x}".format(j, float(time)/timebase, addr, val)
                j += 1
                i += 1
                i %= log_len
            # Write the new tail value.  The simple method does not work.
            #g_reglog['tail'] = head
            head_mv = ReadMemoryCast(g_reglog['tail'].address, 4)
            # Convert to a bytearray so it can be written to.
            head_mb = bytearray(head_mv)
            pack_into('I', head_mb, 0, head)
            WriteMemoryCast(g_reglog['tail'].address, head_mb, 4)

class DumpHexCmd(gdb.Command):
    "Command to dump memory formatted as single byte hex and ascii."

    def __init__(self):
        super (DumpHexCmd, self).__init__("dump hex", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        DumpHexArg('B', arg)

class DumpHexCmdB(gdb.Command):
    "Command to dump memory formatted as single byte hex and ascii."

    def __init__(self):
        super (DumpHexCmdB, self).__init__("dump hexb", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        DumpHexArg('B', arg)

class DumpHexCmdH(gdb.Command):
    "Command to dump memory formatted as two byte hex and ascii."

    def __init__(self):
        super (DumpHexCmdH, self).__init__("dump hexh", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        DumpHexArg('H', arg)

class DumpHexCmdL(gdb.Command):
    "Command to dump memory formatted as four byte hex and ascii."

    def __init__(self):
        super (DumpHexCmdL, self).__init__("dump hexw", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        DumpHexArg('I', arg)

class DumpHexCmdQ(gdb.Command):
    "Command to dump memory formatted as eight byte hex and ascii."

    def __init__(self):
        super (DumpHexCmdQ, self).__init__("dump hexl", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        DumpHexArg('Q', arg)

def SortRegOfs(i):
    return i[1]['ofs']

def SortBitPos(i):
    return i[1]['pos']

def DumpReg(reg_str, reg_adr, size, reg_decode):
    inferior = gdb.selected_inferior()
    # This reads much more than required, it could be optimized.
    try:
        buf = inferior.read_memory(reg_adr, size);
    except:
        raise gdb.GdbError("Unable to read the I2S_SC memory.")

    print reg_str
    for reg, decode in sorted(reg_decode.items(), key=SortRegOfs):
        reg_val = unpack_from("I", buf, decode['ofs'])[0]
        reg_name_str = reg + " ({:08x})".format(reg_adr+decode['ofs'])
        print "  {:<22}: ".format(reg_name_str) + "0x{:08x}".format(reg_val)
        for name, bits in sorted(decode['bits'].items(), key=SortBitPos, reverse=True):
            bit_name_str = name + "[{:d}".format(bits['pos'])
            if (bits['size'] > 1):
                bit_name_str += "...{:d}".format(bits['pos']+bits['size']-1)
            bit_name_str += "]"
            print "    {:<20}: ".format(bit_name_str) + bits['fmt'].format((reg_val>>bits['pos'])&((1<<bits['size'])-1))


class DumpTSBI2SCmd(gdb.Command):
    "Command to dump the contents of TSB I2S registers."

    def __init__(self):
        super (DumpTSBI2SCmd, self).__init__("dump i2s", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        TSB_SYSCTL = 0x40000200
        TSB_I2S_SC = 0x40007000
        TSB_I2S_SO = 0x40008000
        TSB_I2S_SI = 0x40009000
        tsb_so_audioset_fmt = [ 'B', 'B', 'H', 'I' ]
        tsb_sysctl_decode = {
            'CLOCKGATING1' : { 'ofs': 0x004, 'bits':{'CG_i2sBitClk'        : { 'pos': 1, 'size': 1, 'fmt': "0b{:1b}" },
                                                     'CG_i2sSysClk'        : { 'pos': 0, 'size': 1, 'fmt': "0b{:1b}" }}},
            'CLOCKENABLE1' : { 'ofs': 0x104, 'bits':{'CG_i2sBitClk'        : { 'pos': 1, 'size': 1, 'fmt': "0b{:1b}" },
                                                     'CG_i2sSysClk'        : { 'pos': 0, 'size': 1, 'fmt': "0b{:1b}" }}},
            'I2S_CLOCK'    : { 'ofs': 0x240, 'bits':{'I2S_DAC16BIT_SEL'    : { 'pos': 2, 'size': 1, 'fmt': "0b{:1b}" },
                                                     'I2S_LR_BCLK_SEL'     : { 'pos': 1, 'size': 1, 'fmt': "0b{:1b}" },
                                                     'I2S_MASTER_CLOCK_SEL': { 'pos': 0, 'size': 1, 'fmt': "0b{:1b}" }}}
        }
        tsb_sc_decode = {
            'START'    : { 'ofs': 0x04, 'bits' :{'Start'       : { 'pos':  8, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'BUSY'     : { 'ofs': 0x08, 'bits' :{'Busy'        : { 'pos':  8, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'STOP'     : { 'ofs': 0x08, 'bits' :{'I2S_STOP'    : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'AUDIOSET' : { 'ofs': 0x10, 'bits' :{'Edge'        : { 'pos': 11, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SCLKtoWS'    : { 'pos':  8, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'REGBUSY'  : { 'ofs': 0x40, 'bits' :{'MODESETPend' : { 'pos': 19, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'AUDIOSETPend': { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MODESETBusy' : { 'pos':  3, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'AUDIOSETBusy': { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'MODESET'  : { 'ofs': 0xF8, 'bits' :{'WS'          : { 'pos':  0, 'size': 3, 'fmt': "0b{:03b}" }}},
        }
        tsb_so_decode = {
            'START'    : { 'ofs': 0x04, 'bits': {'Start'       : { 'pos':  8, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SpkStart'    : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'BUSY'     : { 'ofs': 0x08, 'bits': {'LRErrBusy'   : { 'pos': 17, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'ErrBusy'     : { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SeriBusy'    : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SpkBusy'     : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'STOP'     : { 'ofs': 0x0C, 'bits': {'I2S_STOP'    : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'AUDIOSET' : { 'ofs': 0x10, 'bits': {'DFTFmt'      : { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SDEdge'      : { 'pos': 12, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'Edge'        : { 'pos': 11, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SCLKtoWS'    : { 'pos':  8, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'WordLen'     : { 'pos':  0, 'size': 6, 'fmt': "0b{:06b}" }}},
            'INTSTAT'  : { 'ofs': 0x14, 'bits': {'LRCKErr'     : { 'pos':  3, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'URErr'       : { 'pos':  2, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'ORErr'       : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'Int'         : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'INTMASK'  : { 'ofs': 0x18, 'bits': {'DMACMSK'     : { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'LRErrMask'   : { 'pos':  3, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'URMask'      : { 'pos':  2, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'ORMask'      : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'Int'         : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'INTCLR'   : { 'ofs': 0x1C, 'bits': {'LRErrClr'    : { 'pos':  3, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'URClr'       : { 'pos':  2, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'ORClr'       : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'IntClr'      : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'MUTE'     : { 'ofs': 0x24, 'bits': {'MuteN'       : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'EPTR'     : { 'ofs': 0x28, 'bits': {'ErrPointer'  : { 'pos':  0, 'size': 6, 'fmt': "0x{:04x}" }}},
            'TX_SSIZE' : { 'ofs': 0x30, 'bits': {'TxStartSize' : { 'pos':  0, 'size': 6, 'fmt': "  {:d}"   }}},
            'REGBUSY'  : { 'ofs': 0x40, 'bits': {'MODESETPend' : { 'pos': 19, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'TXSSizePend' : { 'pos': 18, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MutePend'    : { 'pos': 17, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'AUDIOSETPend': { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MODESETBusy' : { 'pos':  3, 'size': 6, 'fmt': "0b{:1b}"  },
                                                 'TXSSizeBusy' : { 'pos':  2, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MuteBusy'    : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'AUDIOSETBusy': { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'MODESET'  : { 'ofs': 0xF8, 'bits': {'WS'          : { 'pos':  0, 'size': 3, 'fmt': "0b{:03b}" }}},
        }
        tsb_si_decode = {
            'START'    : { 'ofs': 0x04, 'bits': {'Start'       : { 'pos':  8, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MicStart'    : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'BUSY'     : { 'ofs': 0x08, 'bits': {'LRErrBusy'   : { 'pos': 17, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'ErrBusy'     : { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SeriBusy'    : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MicBusy'     : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'STOP'     : { 'ofs': 0x0C, 'bits': {'I2S_STOP'    : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'AUDIOSET' : { 'ofs': 0x10, 'bits': {'DFTFmt'      : { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SDEdge'      : { 'pos': 12, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'Edge'        : { 'pos': 11, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'SCLKtoWS'    : { 'pos':  8, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'WordLen'     : { 'pos':  0, 'size': 6, 'fmt': "0b{:06b}" }}},
            'INTSTAT'  : { 'ofs': 0x14, 'bits': {'LRCKErr'     : { 'pos':  3, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'URErr'       : { 'pos':  2, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'ORErr'       : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'Int'         : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'INTMASK'  : { 'ofs': 0x18, 'bits': {'DMACMSK'     : { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'LRErrMask'   : { 'pos':  3, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'URMask'      : { 'pos':  2, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'ORMask'      : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'Int'         : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'INTCLR'   : { 'ofs': 0x1C, 'bits': {'LRErrClr'    : { 'pos':  3, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'URClr'       : { 'pos':  2, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'ORClr'       : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'IntClr'      : { 'pos':  0, 'size': 6, 'fmt': "0b{:1b}"  }}},
            'MUTE'     : { 'ofs': 0x24, 'bits': {'MuteN'       : { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'EPTR'     : { 'ofs': 0x28, 'bits': {'ErrPointer'  : { 'pos':  0, 'size': 6, 'fmt': "0x{:04x}" }}},
            'REGBUSY'  : { 'ofs': 0x40, 'bits': {'MODESETPend' : { 'pos': 19, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'TXSSizePend' : { 'pos': 18, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MutePend'    : { 'pos': 17, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'AUDIOSETPend': { 'pos': 16, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MODESETBusy' : { 'pos':  3, 'size': 6, 'fmt': "0b{:1b}"  },
                                                 'TXSSizeBusy' : { 'pos':  2, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'MuteBusy'    : { 'pos':  1, 'size': 1, 'fmt': "0b{:1b}"  },
                                                 'AUDIOSETBusy': { 'pos':  0, 'size': 1, 'fmt': "0b{:1b}"  }}},
            'MODESET'  : { 'ofs': 0xF8, 'bits': {'WS'          : { 'pos':  0, 'size': 3, 'fmt': "0b{:03b}" }}},
        }

        inferior = gdb.selected_inferior()
        # This reads much more than required, it could be optimized.
        try:
            buf = inferior.read_memory(TSB_I2S_SO+0x10, 4);
            so_audioset = unpack_from("I", buf, 0)[0]
        except:
            raise gdb.GdbError("Unable to read the SO audio set register.")

        DumpReg("SYC CTL:", TSB_SYSCTL, 0x244, tsb_sysctl_decode)
        DumpReg("I2C SC:", TSB_I2S_SC, 256, tsb_sc_decode)
        DumpReg("I2C SO:", TSB_I2S_SO, 256, tsb_so_decode)
        print "I2C SO TX Data:"
        DumpHex("  ", tsb_so_audioset_fmt[(so_audioset>>3)&0x3], TSB_I2S_SO+0x100, 0x100)
        DumpReg("I2C SI:", TSB_I2S_SI, 256, tsb_si_decode)

DumpSyslogCmd()
DumpRegLogCmd()
DumpHexCmdB()
DumpHexCmdH()
DumpHexCmdL()
DumpHexCmdQ()
DumpTSBI2SCmd()
