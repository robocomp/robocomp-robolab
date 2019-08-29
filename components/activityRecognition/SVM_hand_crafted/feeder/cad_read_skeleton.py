import numpy as np
import os

def read_skeleton(file):
    with open(file, 'r') as f:
        skeleton_sequence = {}

        # num of frames = num of lines - 1, because the last line is 'END' in CAD60
        skeleton_sequence['numFrame'] = len(f.readlines()) -1
        skeleton_sequence['frameInfo'] = []

        # go back to line 0
        f.seek(0)
        # for each frame
        for t in range(skeleton_sequence['numFrame']):
            
            # information about joints can be found in README of CAD-60
            frame_info = {}
            
            frame_info['numBody'] = 1
            frame_info['numJoint'] = 15
            
            # first 11 joints have 9 values for orientation + conf value, and 3 position values + conf value
            # last 4 joints have 3 values for position + conf value
            numJoints_withOri = 11
            numJoints_withoutOri = 4
            attr_joints_withOri = 14
            attr_joints_withoutOri = 4
            
            # we will split the content of each frame into two parts depending on information about joints
            length_firstPart = numJoints_withOri * attr_joints_withOri
            length_secondPart = numJoints_withoutOri * attr_joints_withoutOri

            # every frame information is saved as one line in CAD-60
            cont = f.readline().split(',')
            # first number is the frame's number
            cont_1 = cont[1:length_firstPart + 1]
            # last char is \n, ignore it
            cont_2 = cont[length_firstPart + 1: -1]

            frame_info['jointInfo'] = []

            for j in range(numJoints_withOri):
                joint_info_key = [
                'o1', 'o2', 'o3', 'o4', 'o5', 'o6', 'o7', 'o8', 'o9', 'confO', 'x', 'y', 'z', 'confP' 
                ]
                joint_info = {
                    k: float(v)
                    for k, v in zip(joint_info_key, cont_1[(
                        j * attr_joints_withOri) : (j * attr_joints_withOri) + attr_joints_withOri])
                }
                frame_info['jointInfo'].append(joint_info)
            for j in range(numJoints_withoutOri):
                joint_info_key = [
                    'x', 'y', 'z', 'confP'
                ]
                joint_info = {
                    k: float(v)
                    for k, v in zip(joint_info_key, cont_2[(
                        j * attr_joints_withoutOri) : (j * attr_joints_withoutOri) + attr_joints_withoutOri])
                }
                frame_info['jointInfo'].append(joint_info)
            skeleton_sequence['frameInfo'].append(frame_info)
    return skeleton_sequence


def read_xyz(file, max_body=1, num_joint=15):
    seq_info = read_skeleton(file)
    data = np.zeros((3, seq_info['numFrame'], num_joint))

    if max_body == 1 :
        for n, f in enumerate(seq_info['frameInfo']):
            for j, v in enumerate(f['jointInfo']):
                if j < num_joint:
                    # data [xyz, frame, joint]
                    data[:, n, j] = [v['x'], v['y'], v['z']]
                else:
                    pass
    else :
        # not implemented for now
        pass
    return data


if __name__ == '__main__':
    
    # 1 skeleton test
    data_path = '../../cad60_separated/office/data1'
    test_skeleton = '0512175502.txt' 

    data = read_xyz(os.path.join(data_path, test_skeleton))
    print(data)
    print(data.shape)