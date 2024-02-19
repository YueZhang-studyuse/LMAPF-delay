import random
import json

def generate_delay(p):
    #map_name = 'random'
    number_agents = 5000
    simulation_time = 1000
    #p = 0.2
    file_name = 'lifelong_benchmark/delay2/'+'delay2-'+str(p)+'-1.txt'

    delay = []
    cnt_delay = 0

    for i in range(0,number_agents):
        agent_delay = []
        for j in range(0,simulation_time):
            dp = random.random()
            if dp > p:
                agent_delay.append(str(0))
            else:
                agent_delay.append(str(1))
        delay.append(agent_delay)
        cnt_delay+= agent_delay.count('1')

    with open(file_name,'w') as f:
        f.write(str(number_agents)+ ','+ str(cnt_delay) + '\n')
        for a in delay:
            #print(a)
            f.write(','.join(a) + '\n')
        f.close()



generate_delay(0.01)
generate_delay(0.02)
generate_delay(0.03)
generate_delay(0.04)
generate_delay(0.05)
agent_group = [100,200,300,400]
for agent in agent_group:
    temp = {'mapFile':'maps/random-32-32-20.map',
            'agentFile':'agents/agent-'+str(agent)+'_scen-1.agents',
            'teamSize':agent,
            'taskFile':'tasks/agent-'+str(agent)+'_scen-1.tasks',
            'numTasksReveal':1,
            'taskAssignmentStrategy':'roundrobin',
            "delayFile":"../delay/delay2-0.01-1.txt"}
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay2-0.01-1.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay/delay2-0.02-1.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay2-0.02-1.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay/delay2-0.03-1.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay2-0.03-1.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay/delay2-0.04-1.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay2-0.04-1.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay/delay2-0.05-1.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay2-0.05-1.json','w') as f:
        json.dump(temp,f,indent=2)

agent_group = [200,400,600,800,1000]
for agent in agent_group:
    temp = {'mapFile':'maps/warehouse-10-20-10-2-1.map',
            'agentFile':'agents/agent-'+str(agent)+'_scen-1.agents',
            'teamSize':agent,
            'taskFile':'tasks/agent-'+str(agent)+'_scen-1.tasks',
            'numTasksReveal':1,
            'taskAssignmentStrategy':'roundrobin',
            "delayFile":"../delay/delay2-0.01-1.txt"}
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay2-0.01-1.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay/delay2-0.02-1.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay2-0.02-1.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay/delay2-0.03-1.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay2-0.03-1.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay/delay2-0.04-1.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay2-0.04-1.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay/delay2-0.05-1.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay2-0.05-1.json','w') as f:
        json.dump(temp,f,indent=2)