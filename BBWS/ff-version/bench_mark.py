import  os
import csv
import re
files_list=[]
path = "../ipc-2011-duality/blocksworld/instances_test"
files = os.listdir(path)
s = []
for file in files:
     if not os.path.isdir(file):
          files_list.append(file)

result_dict={}
duality_result_dict={}

print(files_list)
#files_list=["duality_blocksaips01.pddl","blocksaips01.pddl"]

for file in files_list:
    if"duality" in file:
        duality_result_dict[file]={}
        result=os.popen("./bfws --domain  ../ipc-2011-duality/blocksworld/domain_duality.pddl --problem ../ipc-2011-duality/blocksworld/instances_test/"+file+" --output ./result/"+file+"_result.txt --BFWS-f5 1").read()
        for line in result.split("\n"):
            if "Total time" in line:
                duality_result_dict[file]["Total time"]=line.split(":")[-1]
            if "Nodes generated during search" in line:
                duality_result_dict[file]["Nodes generated during search"] = line.split(":")[-1]
            if "Nodes expanded during search" in line:
                duality_result_dict[file]["Nodes expanded during search"] = line.split(":")[-1]
            if "Plan found with cost" in line:
                duality_result_dict[file]["Plan found with cost"] = line.split(":")[-1]
            if "Fast-BFS search completed in" in line:
                duality_result_dict[file]["Fast-BFS search completed in"] = " ".join(line.split(" ")[-2:])
    else:
        result=os.popen("./bfws --domain  ../ipc-2011-duality/blocksworld/domain.pddl --problem ../ipc-2011-duality/blocksworld/instances_test/"+file+" --output ./result/"+file+"_result.txt --BFWS-f5 1 ").read()
        result_dict[file] = {}
        for line in result.split("\n"):
            if "Total time" in line:
                result_dict[file]["Total time"]=line.split(":")[-1]
            if "Nodes generated during search" in line:
                result_dict[file]["Nodes generated during search"] = line.split(":")[-1]
            if "Nodes expanded during search" in line:
                result_dict[file]["Nodes expanded during search"] = line.split(":")[-1]
            if "Plan found with cost" in line:
                result_dict[file]["Plan found with cost"] = line.split(":")[-1]
            if "Fast-BFS search completed in" in line:
                result_dict[file]["Fast-BFS search completed in"] =" ".join(line.split(" ")[-2:])

print(result_dict)
print(duality_result_dict)

result_out_put={}
duality_out_put={}
out_put_list=[]


result_dict_keys=sorted(result_dict.keys())
for key in result_dict_keys:
    result_out_put[key]=list(result_dict[key].values())

for key in duality_result_dict.keys():
     duality_out_put[key.split("_")[1]]=list(duality_result_dict[key].values())

for key in result_out_put.keys():
    temp_list=[]
    temp_list.append(key)
    for i in range(len(result_out_put[key])):
        temp_list.append(result_out_put[key][i])
        temp_list.append(duality_out_put[key][i])
    out_put_list.append(temp_list)

def write(output_list,output_file):
    with open(output_file,"w") as f:
        writer=csv.writer(f)
        for row in output_list:
            writer.writerow(row)


write(out_put_list,"./result/test_result.csv")

#result=os.popen("./bfws --domain  ../ipc-2011-duality/blocksworld/domain_duality.pddl --problem ../ipc-2011-duality/blocksworld/instances/duality_blocksaips01.pddl --output ./result/duality_01_result.txt --BFWS-f5 1").read()

#for line in result.split("\n"):
#     if "Total time" in line: