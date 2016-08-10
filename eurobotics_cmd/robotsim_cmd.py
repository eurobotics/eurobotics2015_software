#! /usr/bin/env python

import os,sys,termios,atexit
#import serial
from select import select
import cmd
#import pylab
from  matplotlib import pylab
from math import *
from subprocess import call

import popen2
from subprocess import Popen, PIPE
from fcntl import fcntl, F_GETFL, F_SETFL
from os import O_NONBLOCK, read

import struct
import numpy
import shlex
import time
import math
import warnings
warnings.filterwarnings("ignore","tempnam",RuntimeWarning, __name__)

import logging
log = logging.getLogger("EuroboticsShell")
_handler = logging.StreamHandler()
_handler.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
log.addHandler(_handler)
log.setLevel(1)

EUROBOTICS_PATH=os.path.dirname(sys.argv[0])

#class SerialLogger:
#    def __init__(self, ser, filein, fileout=None):
#        self.ser = ser
#        self.filein = filein
#        self.fin = open(filein, "a", 0)
#        if fileout:
#            self.fileout = fileout
#            self.fout = open(fileout, "a", 0)
#        else:
#            self.fileout = filein
#            self.fout = self.fin
#    def fileno(self):
#        return self.ser.fileno()
#    def read(self, *args):
#        res = self.ser.read(*args)
#        self.fout.write(res)
#        return res
#    def write(self, s):
#        self.fout.write(s)
#        self.ser.write(s)

class Interp(cmd.Cmd):
    prompt = "Eurobotics > "

    def __init__(self):
        cmd.Cmd.__init__(self)

        o, i = popen2.popen2("python ../maindspic/display.py")
        o.close()
        i.close()

        o, i = popen2.popen2("../secondary_robot/main H=1")
        o.close()
        i.close()

        self.p = Popen(['../maindspic/main', 'H=1'],
                  stdin = PIPE, stdout = PIPE, stderr = PIPE, shell = False)

        # set the O_NONBLOCK flag of p.stdout file descriptor:
        #flags = fcntl(self.p.stdout, F_GETFL) # get current p.stdout flags
        #fcntl(self.p.stdout, F_SETFL, flags | O_NONBLOCK)

        self.escape  = "\x01" # C-a
        self.quitraw = "\x02" # C-b
        self.serial_logging = False
        self.default_in_log_file = "/tmp/eurobotics.in.log"
        self.default_out_log_file = "/tmp/eurobotics.out.log"

    def do_quit(self, args):
        return True

#    def do_log(self, args):
#        """Activate serial logs.
#        log <filename>           logs input and output to <filename>
#        log <filein> <fileout>   logs input to <filein> and output to <fileout>
#        log                      logs to /tmp/eurobotics.log or the last used file"""

#        if self.serial_logging:
#            log.error("Already logging to %s and %s" % (self.ser.filein,
#                                                        self.ser.fileout))
#        else:
#            self.serial_logging = True
#            files = [os.path.expanduser(x) for x in args.split()]
#            if len(files) == 0:
#                files = [self.default_in_log_file, self.default_out_log_file]
#            elif len(files) == 1:
#                self.default_in_log_file = files[0]
#                self.default_out_log_file = None
#            elif len(files) == 2:
#                self.default_in_log_file = files[0]
#                self.default_out_log_file = files[1]
#            else:
#                print "Can't parse arguments"

#            self.ser = SerialLogger(self.ser, *files)
#            log.info("Starting serial logging to %s and %s" % (self.ser.filein,
#                                                               self.ser.fileout))


#    def do_unlog(self, args):
#        if self.serial_logging:
#            log.info("Stopping serial logging to %s and %s" % (self.ser.filein,
#                                                               self.ser.fileout))
#            self.ser = self.ser.ser
#            self.serial_logging = False
#        else:
#            log.error("No log to stop")


#    def do_raw(self, args):
#        "Switch to RAW mode"
#        stdin = os.open("/dev/stdin",os.O_RDONLY)
#        stdout = os.open("/dev/stdout",os.O_WRONLY)

#        stdin_termios = termios.tcgetattr(stdin)
#        raw_termios = stdin_termios[:]

#        try:
#            log.info("Switching to RAW mode")

#            # iflag
#            raw_termios[0] &= ~(termios.IGNBRK | termios.BRKINT |
#                                termios.PARMRK | termios.ISTRIP |
#                                termios.INLCR | termios.IGNCR |
#                                termios.ICRNL | termios.IXON)
#            # oflag
#            raw_termios[1] &= ~termios.OPOST;
#            # cflag
#            raw_termios[2] &= ~(termios.CSIZE | termios.PARENB);
#            raw_termios[2] |= termios.CS8;
#            # lflag
#            raw_termios[3] &= ~(termios.ECHO | termios.ECHONL |
#                                termios.ICANON | termios.ISIG |
#                                termios.IEXTEN);

#            termios.tcsetattr(stdin, termios.TCSADRAIN, raw_termios)

#            mode = "normal"
#            while True:
#                ins,outs,errs=select([stdin,self.robot_out],[],[])
#                for x in ins:
#                    if x == stdin:
#                        c = os.read(stdin,1)
#                        if mode  == "escape":
#                            mode =="normal"
#                            if c == self.escape:
#                                self.fout.write(self.escape)
#                            elif c == self.quitraw:
#                                return
#                            else:
#                                self.fout.write(self.escape)
#                                self.fout.write(c)
#                        else:
#                            if c == self.escape:
#                                mode = "escape"
#                            else:
#                                self.fout.write(c)
#                    elif x == self.robot_out:
#                        os.write(stdout,self.fin.readline())
#        finally:
#            termios.tcsetattr(stdin, termios.TCSADRAIN, stdin_termios)
#            log.info("Back to normal mode")

#    def do_centrifugal(self, args):
#        try:
#            sa, sd, aa, ad = [int(x) for x in shlex.shlex(args)]
#        except:
#            print "args: speed_a, speed_d, acc_a, acc_d"
#            return
#        print sa, sd, aa, ad
#        time.sleep(10)
#        self.fout.write("traj_speed angle %d\n"%(sa))
#        time.sleep(0.1)
#        self.fout.write("traj_speed distance %d\n"%(sd))
#        time.sleep(0.1)
#        self.fout.write("traj_acc angle %d\n"%(aa))
#        time.sleep(0.1)
#        self.fout.write("traj_acc distance %d\n"%(ad))
#        time.sleep(0.1)
#        self.fout.write("goto da_rel 800 180\n")
#        time.sleep(3)
##        self.fout.flushInput()
#        self.fout.write("position show\n")
#        time.sleep(1)
#        print self.fin.read()

    def do_test(self, args):
        time.sleep(0.1)
        self.p.stdin.write("log type cs on\n")
        self.p.stdin.write("goto d_rel 1000\n")
        time.sleep(0.2)

        while True:
          line = self.p.stdout.readline()
          if line != '':
            #the real code does filtering here
            print line.rstrip()
          else:
            break

        print "done"

    def do_position_show(self, args):
        time.sleep(0.1)
        self.fin.flush()
        time.sleep(0.1)
        #self.fout.write("position show\n")
        #time.sleep(1)
        #print self.fin.readline()

#    def do_tota(self, args):
#        print args
#        time.sleep(1)
#        self.fout.write("position set 0 0 0\n")
#        time.sleep(1)
#        self.fout.write("pwm s3(3C) 250\n")


if __name__ == "__main__":
    try:
        import readline,atexit
    except ImportError:
        pass
    else:
        histfile = os.path.join(os.environ["HOME"], ".eurobotics_history")
        atexit.register(readline.write_history_file, histfile)
        try:
            readline.read_history_file(histfile)
        except IOError:
            pass

    interp = Interp()
    while 1:
        try:
            interp.cmdloop()
        except KeyboardInterrupt:
            print
        except Exception,e:
            l = str(e).strip()
            if l:
                log.exception("%s" % l.splitlines()[-1])
            continue
        break
