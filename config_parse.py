import sys
import numpy as np

def parse_dh_param_file(dh_config_file):
    assert(dh_config_file is not None)
    f_line_contents = None
    with open(dh_config_file, "r") as f:
        f_line_contents = f.readlines()

    assert(f.closed)
    assert(f_line_contents is not None)
    # maybe not the most efficient/clean/etc. way to do this, but should only have to be done once so NBD
    dh_params = np.asarray([line.rstrip().split(',') for line in f_line_contents[1:]])
    # print(dh_params)
    dh_params = dh_params.astype(float)
    # print(dh_params)
    return dh_params


### TODO: parse a pox parameter file
def parse_pox_param_file(dh_config_file):
    assert(dh_config_file is not None)
    f_line_contents = None
    with open(dh_config_file, "r") as f:
        f_line_contents = f.readlines()

    assert(f.closed)
    assert(f_line_contents is not None)
    params = np.asarray([line.rstrip().split(',') for line in f_line_contents[1:]])

    MM = params[0:4]
    M=[]
    for m in MM:
        M.append([float(x) for x in m])
   
    Ssc = params[5:10]
    Sc = []
    for s in Ssc:
        Sc.append([float(x) for x in s])
    
    M_mat = np.asarray(M)
    Screw_mat = np.asarray(Sc)

    return (M_mat, Screw_mat)


if __name__=='__main__':
    print(parse_pox_param_file('rx200_pox.csv'))
