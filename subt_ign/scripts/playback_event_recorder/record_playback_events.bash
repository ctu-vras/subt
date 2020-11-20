#!/bin/bash

# The playback event recorder creates videos for events that are stored
# in the events.yml log file in the state log directory. The script launches
# ign gazebo in playback mode and creates videos using the gui camera video
# recording feature. For each event, it seeks playback to some time before the
# event, moves the camera to the desired location, and starts recording until
# some time after the event.

echo "==================================="
echo "Staring Playback Event Recorder"
echo "==================================="

if [ -z "$1" ]; then
  echo "Usage: bash ./record_playback_events.bash [path_to_log_dir] [resolution] [bitrate]"
  echo "Resolution is entered as 1920x1080 and so on. Default is 1920x1080."
  echo "Bitrate is in bits per second. Default is 8000000."
  exit 0
fi

logDirPath=$1

if [ ! -d "$logDirPath" ]; then
  echo "Directory does not exist: $logDirPath"
  exit 0
fi

resolution="1920x1080"
[ -n "$2" ] && resolution="$2"

bitrate=8000000
[ -n "$3" ] && bitrate="$3"

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tmpDir="tmp_record"

echo "Creating tmp dir for recording: $tmpDir"

if [ -d "$tmpDir" ]; then
  rm -fr $tmpDir
fi

ln -s $logDirPath $tmpDir

width="$(echo "$resolution" | cut -d "x" -f 1)"
height="$(echo "$resolution" | cut -d "x" -f 2)"
echo "Creating tmp GUI config for resolution ${width}x${height}"

(cat <<EOF
<?xml version="1.0"?>

<!-- Window -->
<window>
  <width>${width}</width>
  <height>${height}</height>
  <style
    material_theme="Light"
    material_primary="DeepOrange"
    material_accent="LightBlue"
    toolbar_color_light="#f3f3f3"
    toolbar_text_color_light="#111111"
    toolbar_color_dark="#414141"
    toolbar_text_color_dark="#f3f3f3"
    plugin_toolbar_color_light="#bbdefb"
    plugin_toolbar_text_color_light="#111111"
    plugin_toolbar_color_dark="#607d8b"
    plugin_toolbar_text_color_dark="#eeeeee"
  />
  <menus>
    <drawer default="false">
    </drawer>
  </menus>
</window>

<!-- GUI plugins -->
<plugin filename='GzScene3D' name='3D View'>
  <ignition-gui>
    <title>3D View</title>
    <property type='bool' key='showTitleBar'>false</property>
    <property type='string' key='state'>docked</property>
  </ignition-gui>
  <engine>ogre2</engine>
  <scene>scene</scene>
  <ambient_light>0.4 0.4 0.4</ambient_light>
  <background_color>0.8 0.8 0.8</background_color>
  <camera_pose>-6 0 6 0 0.5 0</camera_pose>
  <record_video>
    <use_sim_time>true</use_sim_time>
    <lockstep>true</lockstep>
    <bitrate>${bitrate}</bitrate>
  </record_video>
</plugin>
EOF
) > gui.config

has_gsettings=0
[ -n "$(which gsettings)" ] && has_gsettings=1
old_auto_maximize="$(gsettings get org.gnome.mutter auto-maximize)"
[ "$?" != 0 ] && has_gsettings=0
[ "${has_gsettings}" = 1 ] && gsettings set org.gnome.mutter auto-maximize false

echo "Starting log playback and video recording"

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$LD_LIBRARY_PATH
sdfName="playback_event_recorder"
ign gazebo -v 4 --gui-config gui.config "$scriptDir/$sdfName.sdf"

echo "Video recording ended. Shutting down playback"

pgrep -f $sdfName | xargs kill -9 &>/dev/null

[ "${has_gsettings}" = 1 ] && gsettings set org.gnome.mutter auto-maximize "${old_auto_maximize}"

videoDir=$(date +%s)
echo "Moving mp4 videos to dir: $videoDir"
mkdir $videoDir
mv *.mp4 $videoDir

# remove tmp dir
if [ -d "$tmpDir" ]; then
  rm -fr $tmpDir
fi

rm -f gui.config

echo "Done"
