import random
import json

def generate_delay(p):
    #map_name = 'random'
    number_agents = 5000
    simulation_time = 1000
    #p = 0.2
    file_name = 'lifelong_benchmark/delay-2/'+'delay-'+str(p)+'-2.txt'

    delay = []

    cnt_delay = 0

    for i in range(0,number_agents):
        agent_delay = []
        for j in range(0,simulation_time):
            dp = random.random()
            if dp > p:
                agent_delay.append(str(0))
            else:
                agent_delay+=[str(1)]*(random.randint(1,2))
        delay.append(agent_delay)
        cnt_delay+= agent_delay[0:1000].count('1')
    
    

    with open(file_name,'w') as f:
        f.write(str(number_agents) + ','+ str(cnt_delay) + '\n')
        for a in delay:
            #print(a)
            print(len(a[0:1000]))
            f.write(','.join(a[0:1000]) + '\n')
        f.close()


# generate_delay(0.005)
# generate_delay(0.01)
# generate_delay(0.015)
# generate_delay(0.002)
# generate_delay(0.003)
# generate_delay(0.004)
# generate_delay(0.001)
# generate_delay(0.006)
# generate_delay(0.007)
# generate_delay(0.008)
# generate_delay(0.009)

agent_group = [100,200,300,400,500,600,700]
for agent in agent_group:
    temp = {'mapFile':'maps/random-32-32-20.map',
            'agentFile':'agents/agent-'+str(agent)+'_scen-1.agents',
            'teamSize':agent,
            'taskFile':'tasks/agent-'+str(agent)+'_scen-1.tasks',
            'numTasksReveal':1,
            'taskAssignmentStrategy':'roundrobin',
            "delayFile":"../delay-2/delay-0.001-2.txt"}
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.001-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.002-2.txt"
    with open('lifelong_benchmark/random/agent-1'+str(agent)+'_scen-delay-0.002-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.003-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.003-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.004-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.004-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.005-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.005-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.006-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.006-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.007-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.007-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.008-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.008-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.009-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.009-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.010-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.010-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.015-2.txt"
    with open('lifelong_benchmark/random/agent-'+str(agent)+'_scen-delay-0.015-2.json','w') as f:
        json.dump(temp,f,indent=2)

agent_group = [200,400,600,800,1000,1200,1400]
for agent in agent_group:
    temp = {'mapFile':'maps/warehouse-10-20-10-2-1.map',
            'agentFile':'agents/agent-'+str(agent)+'_scen-1.agents',
            'teamSize':agent,
            'taskFile':'tasks/agent-'+str(agent)+'_scen-1.tasks',
            'numTasksReveal':1,
            'taskAssignmentStrategy':'roundrobin',
            "delayFile":"../delay-2/delay-0.001-2.txt"}
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.001-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.002-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.002-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.003-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.003-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.004-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.004-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.005-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.005-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.006-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.006-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.007-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.007-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.008-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.008-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.009-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.009-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.010-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.010-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.015-2.txt"
    with open('lifelong_benchmark/warehouse-s/agent-'+str(agent)+'_scen-delay-0.015-2.json','w') as f:
        json.dump(temp,f,indent=2)

agent_group = [200,400,600,800,1000,1200,1400]
for agent in agent_group:
    temp = {'mapFile':'maps/ht_mansion_n.map',
            'agentFile':'agents/agent-'+str(agent)+'_scen-1.agents',
            'teamSize':agent,
            'taskFile':'tasks/agent-'+str(agent)+'_scen-1.tasks',
            'numTasksReveal':1,
            'taskAssignmentStrategy':'roundrobin',
            "delayFile":"../delay-2/delay-0.001-2.txt"}
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.001-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.002-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.002-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.003-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.003-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.004-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.004-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.005-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.005-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.006-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.006-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.007-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.007-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.008-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.008-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.009-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.009-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.010-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.010-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.015-2.txt"
    with open('lifelong_benchmark/game/agent-'+str(agent)+'_scen-delay-0.015-2.json','w') as f:
        json.dump(temp,f,indent=2)


agent_group = [500,1000,1500,2000,2500,3000,3500]
for agent in agent_group:
    temp = {'mapFile':'maps/Paris_1_256.map',
            'agentFile':'agents/agent-'+str(agent)+'_scen-1.agents',
            'teamSize':agent,
            'taskFile':'tasks/agent-'+str(agent)+'_scen-1.tasks',
            'numTasksReveal':1,
            'taskAssignmentStrategy':'roundrobin',
            "delayFile":"../delay-2/delay-0.001-2.txt"}
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.001-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.002-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.002-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.003-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.003-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.004-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.004-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.005-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.005-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.006-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.006-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.007-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.007-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.008-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.008-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.009-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.009-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.010-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.010-2.json','w') as f:
        json.dump(temp,f,indent=2)
    temp["delayFile"] = "../delay-2/delay-0.015-2.txt"
    with open('lifelong_benchmark/city/agent-'+str(agent)+'_scen-delay-0.015-2.json','w') as f:
        json.dump(temp,f,indent=2)