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
# @brief   ARA Unipro performance non-regression test script
# @author  Patrick Titiano
#
# usage: ./loopback-perf-regression-test.py inputfile1 inputfile2 [-h] [-v]
#                                                      [-o OUTPUTFILE]
#                                                      [-t THRESHOLD]
#
# Compare measurements values from inputfile2 with inputfile1.
#
# 1/ Print warning in console if a regression was detected.
#    Default regression threshold is 5%. You may change it via the '-t' option.
# 2/ Generate spreadsheet (.ods) merging data from input files, and containing
#    tables storing comparison data.
#    Default output filename is a concatenation of input file names.
#    You may change provide a custom name using the '-o' option.
# 3/ Return regression count:
#    0: no regression detected
#    >0: regression(s) detected
#    <0: failure during script execution
#
# You may use the '-v' option to make execution more verbose (debug purpose).
#
#

from __future__ import print_function
import argparse
import sys
import shutil
import ezodf
import traceback


COLUMN_LIST = ['date',
               'description',
               'operation',
               'device',
               'size',
               'iteration',
               'error',
               'Requests/sec (min)',
               'Requests/sec (max)',
               'Requests/sec (avg)',
               'Requests/sec (jitter)',
               'Total Latency (min)',
               'Total Latency (max)',
               'Total Latency (avg)',
               'Total Latency (jitter)',
               'Greybus-only Latency (min)',
               'Greybus-only Latency (max)',
               'Greybus-only Latency (avg)',
               'Greybus-only Latency (jitter)',
               'Throughput (min)',
               'Throughput (max)',
               'Throughput (avg)',
               'Throughput (jitter)']

ROW_OFFSET_IN1_IN2 = 20
SPEED_COL = 1
OPERATION_COL = 2
THROUGHPUT_JITTER_COL = 22
FIRST_SPEED_ROW = 2
LOOPBACK_OPS = ('sink', 'transfer', 'ping')
FIRST_ROW_OUT = 4
HEADER_COL_IN = 0
HEADER_ROW_IN1 = 1
HEADER_ROW_IN2 = HEADER_ROW_IN1 + ROW_OFFSET_IN1_IN2
HEADER_ROW_DELTA = 42
HEADER_ROW_DELTA_PCT = 63
HEADER_ROW_DELTA_COL = 1
TEMPLATE_ODS_FILE = './loopback-perf-regression-test-template.ods'

verbose = False
warn_threshold = 5 # 5%

def info(*args, **kwargs):
    # Print information-level messages on stdout

    kwargs['file'] = sys.stdout
    print(*args, **kwargs)


def debug(*args, **kwargs):
    # Print debug-level messages on stdout when verbose flag is set

    global verbose
    if verbose == True:
        info(*args, **kwargs)


def err(*args, **kwargs):
    # Print error-level messages on stderr

    kwargs['file'] = sys.stderr
    args = ('error:',) + args
    print(*args, **kwargs)


def compare_figures(filein1, filein2, sheet, speed, measure, val1, val2):
    # Compare 2 values from 2 different cells
    #   - Detect 0 values
    #   - Detect divergence (offset between 2 values > threshold)
    #   - Return regression count

    global warn_threshold
    regcount = 0
    if measure == 'error':
        if val1 != 0:
            info('WARNING: transmission error(s) detected! '
                 '(file: {}, sheet={}, speed=\'{}\', error={})'.format(
                        filein1, sheet, speed, val1))
            regcount += 1
        if val2 != 0:
            info('WARNING: transmission error(s) detected! '
                 '(file: {}, sheet={}, speed=\'{}\', error={})'.format(
                        filein2, sheet, speed, val2))
            regcount += 1
    else:
        if val1 == 0:
            info('WARNING: measure \'{}\' == 0 detected! '
                 '(file: {}, sheet={}, speed=\'{}\')'.format(measure, filein1,
                                                             sheet, speed))
            regcount += 1
        if val2 == 0:
            info('WARNING: measure \'{}\' == 0 detected! '
                 '(file: {}, sheet={}, speed=\'{}\')'.format(measure, filein2,
                                                             sheet, speed))
            regcount += 1
        if regcount != 0:
            return regcount

        delta = val2 - val1
        delta_pct = int((float(delta) / float(val1)) * 100)
        if delta_pct >= warn_threshold:
            info('WARNING: regression on \'{}\' measure detected!'
                 '(sheet={}, speed=\'{}\', off by {}%)'.format(measure,
                                                   sheet, speed,
                                                   delta_pct))
            regcount += 1

    return regcount


def compare(filein1, filein2, fileout):
    # 1/ Copy data from input spreadsheets into comparison spreadsheet
    # 2/ Analyze performance differences to detect regression(s)
    # 3/ Return regression count

    global verbose

    debug('Input Files:\n  {}\n  {}'.format(filein1, filein1))
    debug('Output File:\n  {}'.format(fileout))

    # Open input files
    try:
        spreadsheets = {}
        sheets = {}
        spreadsheets['in1'] = ezodf.opendoc(filein1)
        sheets['in1'] = spreadsheets['in1'].sheets
        spreadsheets['in2'] = ezodf.opendoc(filein2)
        sheets['in2'] = spreadsheets['in2'].sheets
    except Exception as e:
        err("Failed to read input files!")
        if verbose:
            traceback.print_exc()
        raise e

    # Get some info from input spreadsheets and make sure we compare the same
    try:
        if len(sheets['in1']) != len(sheets['in2']):
            err('Input sheet count do not match! ({}) ({})'. format(
                len(sheets['in1']), len(sheets['in2'])))
        sheetcount = len(sheets['in1']) - 1 # discard last sheet titled 'Data'
        debug('Sheet count: {}'.format(sheetcount))

        speedcount = {'in1': 0, 'in2': 0}
        for i in ['in1', 'in2']:
            for r in range(FIRST_SPEED_ROW, sheets[i][0].nrows()):
                operation_cell = sheets[i][0][r, OPERATION_COL]
                if operation_cell.value not in LOOPBACK_OPS:
                    break
                else:
                    speedcount[i] += 1
        if speedcount['in1'] != speedcount['in2']:
             err('Input sheet speed count do not match! ({}) ({})'. format(
                speedcount['in1'], speedcount['in2']))
        speedcount = speedcount['in1']
        debug('Speed count: {}'.format(speedcount))
    except Exception as e:
        err("Failed to retrieve infos from input files!")
        if verbose:
            traceback.print_exc()
        raise e

    # Clone output spreadsheet from template, and open it
    try:
        shutil.copy(TEMPLATE_ODS_FILE, fileout)
        # Retrieve .ods file sheets
        spreadsheets['out'] = ezodf.opendoc(fileout)
        sheets['out'] = spreadsheets['out'].sheets
    except Exception as e:
        err("Failed to clone charts spreadsheet from template!")
        if verbose:
            traceback.print_exc()
        raise e

    # Template spreadsheet has only 1 sheet. Duplicate it to match sheetcount
    try:
        debug('\nDuplicating sheets...')
        for i in range(1, sheetcount):
            duplicate = sheets['out'][0].copy(newname=sheets['in1'][i].name)
            sheets['out'] += duplicate
            # Add input filename to data tables header
            sheets['out'][i][HEADER_ROW_IN1, HEADER_COL_IN].set_value(filein1)
            sheets['out'][i][HEADER_ROW_IN2, HEADER_COL_IN].set_value(filein2)
            debug('Added {} sheet.'.format(sheets['in1'][i].name))
        for i in range(sheetcount):
            # Add input filename to data tables header
            sheets['out'][i][HEADER_ROW_IN1, HEADER_COL_IN].set_value(filein1)
            sheets['out'][i][HEADER_ROW_IN2, HEADER_COL_IN].set_value(filein2)
            sheets['out'][i][HEADER_ROW_DELTA, HEADER_ROW_DELTA_COL].set_value(
                    '{} vs {}'.format(filein2, filein1))
            sheets['out'][i][HEADER_ROW_DELTA_PCT, HEADER_ROW_DELTA_COL].set_value(
                    '{} vs {} (%)'.format(filein2, filein1))
    except Exception as e:
        err("Failed to duplicate sheets in output spreadsheet!")
        if verbose:
            traceback.print_exc()
        raise e

    # Fill tables with data and highlight potential regression
    try:
        debug('\nFilling tables with data...')
        cell_src = {}
        cell_dst = {}
        row_out = {}
        src_val = {}
        regcount = 0
        for s in range(0, sheetcount):
            curr_sheet = sheets['out'][s]
            debug('Sheet: {}'.format(curr_sheet.name))
            for r in range(speedcount):
                row_in = FIRST_SPEED_ROW + r
                row_out['in1'] = FIRST_ROW_OUT + r
                row_out['in2'] = row_out['in1'] + ROW_OFFSET_IN1_IN2
                speed = sheets['in1'][0][row_in, SPEED_COL].value
                debug('Speed (row in:{} row_out:{}): {}'.format(row_in,
                                                                row_out,
                                                                speed))
                for c in range(THROUGHPUT_JITTER_COL + 1):
                    cell_src['in1'] = sheets['in1'][s][row_in, c]
                    cell_src['in2'] = sheets['in2'][s][row_in, c]
                    cell_dst['in1'] = sheets['out'][s][row_out['in1'], c]
                    cell_dst['in2'] = sheets['out'][s][row_out['in2'], c]
                    if c < 4:
                        src_val['in1'] = cell_src['in1'].value
                        src_val['in2'] = cell_src['in2'].value
                    else:
                        src_val['in1'] = int(cell_src['in1'].value)
                        src_val['in2'] = int(cell_src['in2'].value)
                        regcount += compare_figures(filein1, filein2,
                                                    curr_sheet.name,
                                                    speed,
                                                    COLUMN_LIST[c],
                                                    src_val['in1'],
                                                    src_val['in2'])
                    cell_dst['in1'].set_value(cell_src['in1'].value)
                    cell_dst['in2'].set_value(cell_src['in2'].value)
    except Exception as e:
        debug('c={} val1={} val2={}'.format(c, cell_src['in1'].value,
                                            cell_src['in2'].value))
        err("Failed to fill spreadsheet tables with data!")
        if verbose:
            traceback.print_exc()
        # Save spreadsheet
        spreadsheets['out'].save()
        raise e

    # Save spreadsheet
    spreadsheets['out'].save()
    return regcount

#
# main
#
def main():
    global verbose
    global warn_threshold

    info('ARA Unipro performance non-regression test script\n')

    # Parse arguments.
    parser = argparse.ArgumentParser()
    parser.add_argument('inputfiles', help='Spreadsheet input Files', nargs=2)
    parser.add_argument('-v', '--verbose', dest="verbose", default=False,
                        action='store_true',
                        help='Make script execution more verbose')
    parser.add_argument('-o', '--outputfile', nargs=1,
                        help='Custom output spreadsheet file')
    parser.add_argument('-t', '--threshold',
                        help='Regression warning Threshold')
    args = parser.parse_args()
    verbose = args.verbose
    if args.outputfile:
        fileout = args.outputfile[0]
    else:
        fileout = '{}_{}.ods'.format(args.inputfiles[0].split('.ods', 1)[0],
                                     args.inputfiles[1].split('.ods', 1)[0])
    if args.threshold:
        warn_threshold = int(args.threshold)
        info('NB: regression warning threshold changed to {}%.\n'.format(
             warn_threshold))
    else:
        info('NB: using default warning threshold ({}%).\n'.format(
             warn_threshold))

    info('Comparing \'{}\' vs \'{}\'...\n'.format(
         args.inputfiles[0], args.inputfiles[1]))
    try:
        ret = compare(args.inputfiles[0], args.inputfiles[1], fileout)
    except Exception as e:
        err('unexpected error while comparing files!\n')
        raise e
    info('\nRegression test results:')
    if ret == 0:
        info('  PASSED. No regression detected :-)\n')
    else:
        info('  FAILED! {} regression(s) detected :-(\n'.format(ret))
    info('\nRegression test data saved in \'{}\' spreadsheet.\n'.format(
         fileout))


if __name__ == '__main__':
    main()
