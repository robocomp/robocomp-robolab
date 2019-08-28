## Requirements

* python 3
* numpy

## Data separation (run if you want to do evaluation for each environment separately)

CAD-60 dataset divides all data into 5 groups, which they call environments. To each environment only certain number of activities belong. The data itself comes in 4 folders according to the 4 subjects performing the activities.
However the training and testing is performed per environment.  
To seperate the data into separate directories according to the environment group, the data_separation_script can be used.

running the following command will create 5 directories for each environment. inside of each environment there will be 4 directories for each subject. skeleton files will be distributed accordingly. common label map will be saved at the root of the directory where the seperated dataset is stored.

```commandline
python support_operations/data_separation_script.py --dataset_dir folder_with_original_data --separated_dataset_dir target_folder
```  

for example:
```commandline
python support_operations/data_separation_script.py --dataset_dir ../cad60dataset --separated_dataset_dir ../separated_cad60
```
