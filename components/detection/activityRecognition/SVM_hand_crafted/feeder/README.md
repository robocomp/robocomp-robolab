## Requirements

* python 3
* numpy

## Running the code

0. change to the feeder director

```commandline
cd feeder
```

1. run cad_gendata to save skeletons from txt to python structures
```commandline
python cad_gendata.py --data_path your-path-to-cad-60-dataset(full or separated by environments)
```

example on my computer:
```commandline
python cad_gendata.py --data_path ../../cad60dataset
```
This will save npy and pickle file in folder 'data0' in the same directory as 'feeder' folder

2. As a test you can run feeder to load the npy and pickle files and into feeder class

```commandline
python feeder.py
```

This will print the list of all samples with their labels and other statistics