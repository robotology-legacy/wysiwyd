To transfer an instance from one ABM to another, do the following:
1) Copy the images within the "ABMStoring/tmp/#InstanceNumber" to the other ABM; make sure you use Linux file systems as FAT does not support ":" in the file name
2) Within PgAdmin, do "select * from visualdata where instance=#InstanceNumber"
3) create table export_table_#InstanceNumber as 
select *
from visualdata
where instance = #InstanceNumber
4) Rightclick on export_table_#InstanceNumber then select "Backup", change format to "Plain" and set file name, in "Dump Options #2" tick "Use Column Inserts"
5) In the exported file, search for "INSERT INTO export_table_#InstanceNumber" and replace with "INSERT INTO visualdata"
6) Insert a new instances to the ABM, remember to replace TODAYSTIME and old_instance+1: INSERT INTO main(activityname, activitytype, time, instance, begin, opcname) VALUES ('action', 'action', 'TODAYSTIME', old_instance+1, TRUE, 'OPC')
7) In the exported file, search for ", #InstanceNumber, " (obviously use an actual instance instead of #InstanceNumber) and replace with ", new_instance, "; remember not to change the instance in the filepath!
8) Copy all INSERT INTO statements in PgAdmin of the new ABM; don't copy the first lines starting with "SET"
9) Test and let me (t.fischer@imperial.ac.uk) know if something didn't work :)

