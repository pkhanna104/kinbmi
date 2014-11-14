#/bin/sh

#YEAR=$(date -d "$D" '+%y')
#MONTH=$(date -d "$D" '+%m')
#DATE=$(date -d "$D" '+%d')
TAG=$1
TAG2=${TAG:4}
echo $TAG2

#BMI DATA
rsync -v -rl --partial --include="dat$TAG2*.mat" --exclude="*" /home/lab/bmi_folders/Preeya/seba/data/ /backup/seba/$TAG/bmi_data/

rsync -v -rl --partial --include="dec$TAG2*.mat" --exclude="*" /home/lab/bmi_folders/Preeya/seba/decoders/ /backup/seba/$TAG/bmi_data/

echo $TAG

#EXCEL SHEET
rsync -v -rl --partial --include="$TAG*.xls" --exclude="*" /storage/plexon/Seba/$TAG/ /backup/seba/$TAG/


#Plexon Data
rsync -v -rlt --partial --include="$TAG*.plx" --exclude="*" /storage/plexon/Seba/$TAG/ /backup/seba/$TAG/map_data/
