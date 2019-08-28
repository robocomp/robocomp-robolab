
## Transfer Learning

The code is aimed at pre-training on NTU RGB-D dataset and fine-tuning afterwards on CAD-60.  

It is based on PyTorch Reimplementation of HCN
**Co-occurrence Feature Learning from Skeleton Data for Action Recognition and Detection with Hierarchical Aggregation**.
Chao Li, Qiaoyong Zhong, Di Xie, Shiliang Pu, IJCAI 2018. Please check below for details.


## Prerequisites

- Python (>=3.5)
- PyTorch (>=0.4.0)
- [torchnet](https://github.com/pytorch/tnt)
- Visdom
- Pytorch, torchnet and visdom can be installed with pip (i.g. pip install visdom)

## Usage
#### Data preparation

##### NTU RGB+D
To transform raw NTU RGB-D data into numpy array so that the data can be loaded later for training:
```shell
cd feeder
python ntu_gendata_partial_joints.py --data_path <path for raw skeleton dataset> --out_folder <path for new dataset>
```
This data already selects only those joints that also relate to CAD-60 dataset and orders them as in CAD-60.


##### CAD-60
Similarly, to read the CAD-60 txt files and transfor them into numpy array:
```shell
cd feeder
python cad_gendata_all.py --data_path <path for raw skeleton dataset> --out_folder <path for new dataset>
```
Produces 4 sets of training and validation data. Each pair of training and validation data has 3 subjects' data as the training data and 1 subject's data as the validation.

#### Training
1. Before you start the training, you have to launch [visdom](https://github.com/facebookresearch/visdom) server.
The command is simply:
```shell
visdom
```
After that the training information such as loss in real-time can be observed at the link that will be shown in the shell. If the code is not changed, it should be http://localhost:8097/.

To train the model, you should note that:
 - ```--dataset_dir``` is the **parents path** for **all** the datasets. By default the code creates ```data0``` directory and puts the generated numpy data files there.
 - ``` --num ``` the number of experiment trials. Use it to differentiate training attempts.

2. 
```shell
python main.py --dataset_dir <parents path for all the datasets> --mode train --model_name HCN --dataset_name NTU-RGB-D-CS --num 01
```
When you run the command for the first time, it will create a new directory for training in the experiments directory, but will throw an error because it cannot find params.json, which is a file with training parameters. Simply copy one of the params.json files provided in other experiments folder in this repository to the newly created directory and adapt it to your needs. Re-run the command.

example of command:
```shell
python main.py --dataset_dir data0 --mode train --model_name HCN --dataset_name NTU-RGB-D-CS --num 01
```

#### Testing
To run the test only without training run:
```shell
python main.py --dataset_dir <parents path for all the datasets> --mode test --load True --model_name HCN --dataset_name NTU-RGB-D-CV --num 01
```
for CAD-60 don't forget to add --cad_fold parameter:
examples commands for NTU and CAD:
```shell
python main.py --dataset_dir data0 --mode test --load True --model_name HCN --dataset_name NTU-RGB-D-CS --num 06
```
```shell
python main.py --dataset_dir data0 --mode test --load True --model_name HCN --dataset_name CAD-60 --cad_fold 1  --num 18
```

#### Fine-tuning CAD-60 on the pre-trained model
You can load a NTU pre-trained model (pth.tar checkpoint from the experiments folder), and continue training it on CAD-60. --cad_fold parameter allows to select the subject whose data will be used for validation:
```shell
python main.py --dataset_dir <parents path for all the datasets> --mode load_train --load True --model_name HCN --dataset_name CAD-60  --cad_fold validation-subject --num 01 --load_model <path for  trained model>
```
example of command:
```shell
python main.py --dataset_dir data0 --mode load_train --load True --model_name HCN --dataset_name CAD-60 --cad_fold 1 --num 01 --load_model experiments/NTU-RGB-D-CS/HCN06/checkpoint/best.pth.tar 
```

## Results

transfer_learning.txt contains information about the accuracies achieved for CAD-60. Confusion matrices and loss figures are below.

![Confusion matrices](confusion.png)  

![Losses](losses.png)



## Reference

This code is based on PyTorch Reimplementation of HCN
**Co-occurrence Feature Learning from Skeleton Data for Action Recognition and Detection with Hierarchical Aggregation**.
Chao Li, Qiaoyong Zhong, Di Xie, Shiliang Pu, IJCAI 2018.

[Arxiv Preprint](http://arxiv.org/pdf/1804.06055.pdf)

[original repository](https://github.com/huguyuehuhu/HCN-pytorch)

[1] Chao Li, Qiaoyong Zhong, Di Xie, Shiliang Pu. Co-occurrence Feature Learning from Skeleton Data for Action Recognition and Detection with Hierarchical Aggregation. IJCAI 2018.
https://github.com/huguyuehuhu/HCN-pytorch


