This app is a hack which provides an CONFIG_USER_ENTRYPOINT which is
meant to be useful during bringup. It's not much use for anything
else.

It exists because we're (2015-07-24) doing machine initialization in
the apbridge and gpbridge apps, which we can't do twice. Rather than
clean that up, we're piling on to the problem in order to make a
bringup deadline.

FIXME move machine init into nsh_archinitialize(), then delete this.
