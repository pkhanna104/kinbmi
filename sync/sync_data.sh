#/bin/sh

YEAR=$(date -d "$D" '+%y')
MONTH=$(date -d "$D" '+%m')
DATE=$(date -d "$D" '+%d')
TAG="seba$MONTH$DATE$YEAR"

echo $TAG


#BMI DATA
#rsync -v -rl --partial --include="dat$MONTH$DATE$YR*.mat" --exclude="*" /home/lab/bmi_folders/Preeya/seba/data/ /backup/seba/$TAG/bmi_data/
#rsync -v -rl --partial --include="dec$MONTH$DATE$YR*.mat" --exclude="*" /home/lab/bmi_folders/Preeya/seba/decoders/ /backup/seba/$TAG/bmi_data/

echo $TAG

FILE="/storage/plexon/Seba/$TAG"
FILE1="/backup/seba/$TAG"

echo $FILE
echo $FILE1

#EXCEL SHEET
rsync -v -rl --partial --include="$TAG*.xls" --exclude="*" /storage/plexon/Seba/$TAG/ /backup/seba/$TAG/



#Plexon Data
rsync -v -rlt --partial --include="$TAG*.plx" --exclude="*" /storage/plexon/Seba/$TAG/ /backup/seba/$TAG/map_data/
