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

echo_and_run cd "/home/gzb/catkin_workspace/src/vision_opencv/cv_bridge"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/gzb/catkin_workspace/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/gzb/catkin_workspace/install/lib/python3/dist-packages:/home/gzb/catkin_workspace/build/cv_bridge/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/gzb/catkin_workspace/build/cv_bridge" \
    "/usr/bin/python3" \
    "/home/gzb/catkin_workspace/src/vision_opencv/cv_bridge/setup.py" \
     \
    build --build-base "/home/gzb/catkin_workspace/build/cv_bridge" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/gzb/catkin_workspace/install" --install-scripts="/home/gzb/catkin_workspace/install/bin"
