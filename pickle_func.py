# -*- coding: utf-8 -*-
import pickle
def save_variable(v,filename):
    with open(filename,'wb') as f:
        pickle.dump(v,f)
    return filename
 
def load_variable(filename):
    with open(filename,'rb') as f:
        r=pickle.load(f)
        f.close()
    return r
 
if __name__=='__main__':
    c = [1, 2, 3, 4, 5, 6, 7]
    filename=save_variable(c,"a.txt")
    d=load_variable(filename)
