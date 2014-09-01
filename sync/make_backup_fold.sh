#!/bin/sh

YEAR=$(date -d "$D" '+%y')
MONTH=$(date -d "$D" '+%m')
DATE=$(date -d "$D" '+%d')

anim='seba'
mkdir /backup/$anim/$anim$MONTH$DATE$YEAR
mkdir /backup/$anim/$anim$MONTH$DATE$YEAR/bmi_data
mkdir /backup/$anim/$anim$MONTH$DATE$YEAR/map_data
