#!/usr/bin/env python2
#
#
# Copyright (c) 2015 Google Inc.
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
# @brief   Generate loopback traffic from AP to APB2+APB3 as per
#          SW-912 T2 test case
#
# @author  Patrick Titiano
#
# usage: ./loopback-perf-T2.py HOST [-h] [-r BAUDRATE] [-s SIZE]
#                                        [-i ITERATION] [-v]
#                                        [-o OUTFILENAME]
#
# Use this script to generate a predefined loopback traffic (sink operations)
# from AP module to APB2 and APB3 bridges in parallel.
# This test case is refered as 'T2' in SW-912 documentation.
# Built on top of 'loopback-perf-driver.py' script.
#
# Only mandatory command-line argument is 'HOST': IP address of the AP module.
# Use '-r' option to change the baud-rate of SVC and APB tty
# Use '-s' option to change the size operation payload size.
# Use '-i' option to change the number to iterations to run the test over.
# Use '-o' option to change the name of the generated output files.
# Use '-v' option to make script more verbose (debug purposes).
#
#


from os import system
import argparse
import datetime

LOOPBACK_PERF_DRIVER = './loopback-perf-driver.py'

def main():

    now = datetime.datetime.now()

    # Parse arguments.
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--baudrate',
                        help='baud rate of SVC/APB tty')
    parser.add_argument('host', help='IP/hostname of target AP')
    parser.add_argument('-s', '--size', default=500, help='Packet Size')
    parser.add_argument('-i', '--iteration',
                        help='The number of iterations to run the test over',
                        default=100)
    parser.add_argument('-o', '--output', nargs=1,
                        help='Change the name of the output file names.')
    parser.add_argument('-v', '--verbose', dest="verbose", default=False,
                        action='store_true',
                        help='Make script execution more verbose')
    args = parser.parse_args()

    # Call loopback perf driver with T2-specific options
    cmd = '{} '.format(LOOPBACK_PERF_DRIVER)
    # User command-line options
    cmd += '{} '.format(args.host)
    if args.baudrate:
        cmd += '-r {} '.format(args.baudrate)
    if args.size:
        cmd += '-s {} '.format(args.size)
    if args.iteration:
        cmd += '-i {} '.format(args.iteration)
    if args.output:
        cmd += ' -o {}'.format(args.output[0])
    else:
        cmd += ' -o {}_T2.csv'.format(now.strftime('%Y%m%d-%H%M%S'))
    if args.verbose:
        cmd += ' -v'.format(args.baudrate)
    # 'AP to APB2+APB3' test case specific options ('T2')
    cmd += ' --ap'
    cmd += ' -t sink'
    cmd += ' -b APB2 APB3'

    if args.verbose:
        print(cmd)
    system(cmd)

if __name__ == '__main__':
    main()
