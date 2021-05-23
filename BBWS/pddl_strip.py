import re

from PDDL import PDDL_Parser

from collections import defaultdict
from itertools import combinations

def domain_transfer(domain_name):
    str_action = ""
    str_all = ""
    correct_number = float("inf")
    Flag = False
    left_number = 0
    right_number = 0
    with open(domain_name) as f:
        for line in f:
            line=line.lower()
            str_all+=line
            if ":action" in line:
                Flag=True
                left_number = 0
                right_number = 0
            if correct_number!=0 and Flag==True:
                    str_action+=line
                    for i in line.split():
                        left_number+=i.count("(")
                        right_number+=i.count(")")
                    correct_number=left_number-right_number

            if (correct_number<=0 and Flag==True):
                precondition_str_changed =""
                del_str = ""
                lines=str_action.split("\n")
                effect_words=""
                effect_words_2=""
                flag=False
                precondition_str=""
                precondition_str_2=""
                for line in range(len(lines)):
                    temp_list=[]
                    if ":precondition" in lines[line] and "and" not in lines[line]:
                        precondition_str_changed=re.findall(r"\(.*\)",lines[line])[0]
                        precondition_str = lines[line]
                        precondition_str_2=lines[line]
                        for element in re.findall(r"\(.*\)",precondition_str_changed):
                            temp_list.append("(not "+element+")")
                        precondition_str_changed=" ".join(temp_list)
                        continue
                    if ":precondition" in lines[line] and "and"  in lines[line]:
                        precondition_str_changed=re.findall(r"(?<=and ).*(?=\))",lines[line])[0]
                        precondition_str=lines[line]
                        precondition_str_2 = lines[line]
                        for element in re.findall(r"[(](.*?)[)]",precondition_str_changed):
                            temp_list.append("(not ("+element+"))")
                        precondition_str_changed=" ".join(temp_list)
                        continue
                    if re.search(r'\(*\(\bnot\b.*\)', lines[line]):
                        if "(and" in lines[line]:
                            del_str+=re.findall(r'\(*\(\bnot\b.*\)', lines[line])[0]+"\n"
                        else:
                            del_str+=lines[line]+"\n"
                    if ":effect" in lines[line]:
                        effect_words="\n".join(lines[line+1:])
                        effect_words_2="\n".join(lines[line+1:])
                del_str_left_number = 0
                del_str_right_number = 0
                new_lines=del_str.split("\n")
                for line in new_lines:
                    for i in line.split():
                        del_str_left_number+=i.count("(")
                        del_str_right_number += i.count(")")
                sub_correct_number = del_str_left_number - del_str_right_number
                if sub_correct_number >0:
                    new_lines[-2]=new_lines[-2]+")"*sub_correct_number
                if sub_correct_number <0:
                    new_lines[-2]=new_lines[-2][:sub_correct_number]
                re_construct="\n".join(new_lines[:-1])
                origin_construct=" ".join([re.findall(r"(?<=not ).*(?=\))",i)[0] for i in new_lines[:-1]])
#                origin_construct=" ".join(re_construct.split("\n"))
                new_precondition=precondition_str.replace(precondition_str_changed,origin_construct)
                new_effect_words=effect_words.replace(re_construct,precondition_str_changed)
                precondition_str_all=str_all.replace(precondition_str_2,new_precondition)
                final_str_all=precondition_str_all.replace(effect_words_2,new_effect_words)
                str_all=final_str_all
                correct_number=float("inf")
                str_action = ""
                Flag=False
    f.close()
    return str_all

def problem_transfer(domain_name,problem_name):
    result=""
    goal_state_final=""
    with open(problem_name) as f:
        for lines in f:
            lines=lines.lower()
            if "define "  in lines or ":domain" in lines or ":objects" in lines:
                result+=lines
            else:
                parser = PDDL_Parser()
                parser.parse_domain(domain_name)
                parser.parse_problem(problem_name)
                # Parsed data
                state = parser.state

                goal_pos = parser.positive_goals
                goal_not = parser.negative_goals
                # Do nothing

                iw_name_object = []
                iw_based_states = []


                for iw_object in parser.iw:
                    for iw in iw_object.assign_object(parser.objects):
                        iw_name_object.append([iw.name, iw.parameters])

                for objects in iw_name_object:
                    temp_list = []
                    temp_list.append(objects[0])
                    for object in objects[1]:
                        temp_list.append(object)
                    iw_based_states.append(tuple(temp_list))

                ini = []
                final_goal = []
                for iw in iw_based_states:
                    if list(iw) not in goal_pos:
                        ini.append(list(iw))
                    if list(iw) not in state:
                        final_goal.append(list(iw))
                ini_state=[" ".join(i) for i in ini]
                final_goal_state=[" ".join(i) for i in final_goal]
                ini_state_final="(:init "+" ".join(["("+j+")" for j in ini_state])+")"
                if len(final_goal_state)==1:
                    goal_state_final="(:goal "+" ".join(["("+j+")" for j in final_goal_state])+"))"
                if len(final_goal_state)>1:
                    goal_state_final = "(:goal (and " + " ".join(["(" + j + ")" for j in final_goal_state]) + ")))"
                result+=ini_state_final+"\n"
                result+=goal_state_final
                break
    f.close()
    return result

def wirte(name,string):
    with open(name,"w") as f:
        f.write(string)
    f.close()
if __name__=="__main__":
    import os
    #domain_transfer_str=domain_transfer("./examples/blocksworld/domain_test.pddl")
    #problem__transfer_str=problem_transfer("./examples/blocksworld/domain_test.pddl","./examples/blocksworld/blocksaips01.pddl")
    domain_transfer_str = domain_transfer("../ipc-2011/blocksworld/domain.pddl")
    wirte("../ipc-2011/blocksworld/domain_duality.pddl", domain_transfer_str)
#    problem__transfer_str = problem_transfer("../ipc-2011/blocksworld/domain.pddl","../ipc-2011/blocksworld/instances/blocksaips10.pddl")
#    wirte("../ipc-2011/blocksworld/instances/blocksaips10_duality.pddl", problem__transfer_str)
    #wirte("./examples/blocksworld/domain_test_duality.pddl",domain_transfer_str)
    #wirte("./examples/blocksworld/blocksaips01_duality.pddl",problem__transfer_str)

    path = "../ipc-2011/blocksworld/instances"
    files = os.listdir(path)
    s = []
    for file in files:
        if not os.path.isdir(file):
            problem__transfer_str = problem_transfer("../ipc-2011/blocksworld/domain.pddl",path + "/" + file)
            wirte(path + "/" + "duality_"+file, problem__transfer_str)








