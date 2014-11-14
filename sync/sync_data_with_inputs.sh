#/bin/sh

TAG=$1
MDY=${TAG:4:6}
echo $TAG
echo $MDY


#BMI DATA
rsync -v -rl --partial --include="dat$MDY*.mat" --exclude="*" /home/lab/bmi_folders/Preeya/seba/data/ /backup/seba/$TAG/bmi_data/

rsync -v -rl --partial --include="dec$MDY*.mat" --exclude="*" /home/lab/bmi_folders/Preeya/seba/decoders/ /backup/seba/$TAG/bmi_data/

#EXCEL SHEET
rsync -v -rl --partial --include="$TAG*.xls" --exclude="*" /storage/plexon/Seba/$TAG/ /backup/seba/$TAG/

#Plexon Data
rsync -v -rl --partial --include="$TAG*.plx" --exclude="*" /storage/plexon/Seba/$TAG/ /backup/seba/$TAG/map_data/
