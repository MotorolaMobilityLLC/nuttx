#!/usr/bin/env python

from __future__ import print_function

import subprocess
import sys

def do_rbt_post(revision, depends_on):
    rbt_post = ['rbt', 'post']
    if depends_on is not None:
        rbt_post.extend('--depends-on {}'.format(depends_on).split())
    rbt_post.extend(['-p', revision])
    output = subprocess.check_output(rbt_post)
    print(output,end='')
    return output

def parse_review_num_from_rbt_post_output(output):
    # Parse the review number (in this case 1002) from the rbt post
    # output, which looks like this:
    #
    #   Review request #1002 posted.
    #
    #   https://reviews.leaflabs.com/r/1002/
    #   https://reviews.leaflabs.com/r/1002/diff/
    prefix = 'review request #'
    for line in output.splitlines():
        if line.lower().strip().startswith(prefix):
            return int(line[len(prefix):].split()[0])
    raise ValueError(output)

def post_patch_series(revisions):
    depends_on = None
    for revision in revisions:
        rbt_post_stdout = do_rbt_post(revision, depends_on)
        depends_on = parse_review_num_from_rbt_post_output(rbt_post_stdout)

def usage(file_=sys.stderr):
    print('Usage:', file=file_)
    print('{} <rev1..rev2>'.format(sys.argv[0]), file=file_)
    print(' Post the commits in the given revision range to ReviewBoard,\n'
          '   adding a depends-on field to each review request which is the\n'
          '   previous patch in the series.',
          file=file_)

def main():
    if len(sys.argv) != 2:
        print('A single revision range must be given', file=sys.stderr)
        usage()
        sys.exit(1)
    if '..' not in sys.argv[1] or '...' in sys.argv[1]:
        print('You must specify a revision range rev1..rev2 (but not rev1...rev2)\n',
              'For more information, see:\n',
              'https://www.kernel.org/pub/software/scm/git/docs/gitrevisions.html\n',
              file=sys.stderr)
        usage()
        sys.exit(1)
    revisions = sys.argv[1]
    try:
        rev_list = subprocess.check_output(['git',
                                            'log',
                                            '--reverse',
                                            '--pretty=%H',
                                            revisions]).splitlines()
    except subprocess.CalledProcessError:
        sys.exit(1)
    post_patch_series(rev_list)

if __name__ == '__main__':
    main()
