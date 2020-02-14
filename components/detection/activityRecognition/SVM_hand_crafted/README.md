# Requirements
python (works with both python 2 and python 3)  
numpy  
scikit-learn  
matplotlib

# Use

1. Clone the repository

2. Download [cad-60 dataset](http://pr.cs.cornell.edu/humanactivities/data.php), unzip all 4 parts to one directory

3. [Optional step] If you want to follow evaluation scenario as in the original CAD-60 paper, where dataset is broken down to 5 environments (smaller groups of actions) run data_separation_script.py from support_operations directory. 
for that create a folder where you would want the script to put the sorted dataset: 

```commandline
cd support_operations

python data_separation_script.py --dataset_dir source_folder --separated_dataset_dir target_folder
```
for example:
```commandline
python data_separation_script.py --dataset_dir ../../cad60dataset --separated_dataset_dir ../../separated_cad60
```

If you want to run classification for all 12 classes, just ignore this step and go to step 4 drectly.

4. The next step is to read the skeletons from txt files and save them to python data structures. If you want to run the code for each environment separately, pass the data_path to your separated dataset and indicate "separated" as parameter for --envs. If You want to train and evaluate on all 12 classes at the same time, indicate data_path where the normal unzipped CAD-60 data lies and pass "all" value to argument --envs
```commandline
cd feeder

python cad_gendata.py --data_path your-path-to-cad-60-dataset --envs environment-option
```

for example
```commandline
python cad_gendata.py --data_path ../../cad60dataset --envs all
```

5. Now you can run the main. the default way to run main.py is:

```commandline
python main.py
```
by default it assumes that data (.npy and .pkl files) is saved into data0 folder at the root of the project folder where the main.py exists.
The default run only delivers the accuracy for the cross-validation without confusion matrices or saving the model. Default option also assumes all environments (all 12 classes)

the full command to run main.py with all parameters is:

```commandline
python main.py --dataset-dir data-folder --dataset_name dataset-name --envs environment-option --run run-option
```

the default arguments are:

```commandline
python main.py --dataset-dir ./data0 --dataset_name CAD-60 --envs all --run cv
```

if you want to produce confusion matrices or save the model, run main.py as follows:

```commandline
python main.py --run confusion
```
or

```commandline
python main.py --run final_model
```

the last option trains on all data and saves the model to /models directory

# Credits

[hcn](https://github.com/huguyuehuhu/HCN-pytorch)  
[scikit-learn](https://scikit-learn.org/stable/auto_examples/model_selection/plot_confusion_matrix.html)



