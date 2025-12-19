#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/root/BumbleBot_WS/src/robot_upstart"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/root/BumbleBot_WS/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/root/BumbleBot_WS/install/lib/python3/dist-packages:/root/BumbleBot_WS/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/root/BumbleBot_WS/build" \
    "/usr/bin/python3" \
    "/root/BumbleBot_WS/src/robot_upstart/setup.py" \
     \
    build --build-base "/root/BumbleBot_WS/build/robot_upstart" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/root/BumbleBot_WS/install" --install-scripts="/root/BumbleBot_WS/install/bin"
