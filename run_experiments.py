import subprocess
import os

exp = './exp-delay-our/'
delay_model = ''

mapf_planner = [1,2,3]
mapf_planner = [3]
planner = 3
delay = [0.001,0.002,0.003,0.004,0.005,0.006,0.007,0.008,0.009,0.01,0.015]
delay = [0.01,0.05]
delay_policy = ['mcp','pibt']
policy = [1,2]

commits = [3,10]

for po in policy:
    # #run ramdom
    teams = [i for i in range(100,501,100)]
    for c in commits:
        for d in delay:
            for agent in teams:
                if d == 0.01:
                    d = '0.010'
                if d == 0.05:
                    d = '0.050'
                input_file = './lifelong_benchmark/random/agent-' + str(agent) + '_scen-delay'+delay_model+'-' + str(d) + '-1.json'

                if po == 1:
                    continue
                    output_json = exp + 'random/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
                    output_log = exp + 'random/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
                    output_cout = exp + 'random/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

                    cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(planner) + " --delayPolicy " + str(po) + " --delaySimulateAll true" + " >" + output_cout
                    p = subprocess.Popen(cmd,shell=True)
                    p.wait()
                    print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate all finished')

                output_json = exp + 'random/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
                output_log = exp + 'random/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
                output_cout = exp + 'random/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

                cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(planner) + " --delayPolicy " + str(po) + " --delaySimulateAll false" + " >" + output_cout
                p = subprocess.Popen(cmd,shell=True)
                p.wait()
                print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate window finished')

                output_json = exp + 'random/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
                output_log = exp + 'random/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
                output_cout = exp + 'random/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

                cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(2) + " --delayPolicy " + str(po) + " --delaySimulateAll false" + " >" + output_cout
                p = subprocess.Popen(cmd,shell=True)
                p.wait()
                print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' replan all finished')


    # # #run warehouse-s
    # # #mapf_planner = [1,2,3]

    teams = [i for i in range(200,1401,200)]
    #teams = [i for i in range(200,1001,200)]
    #teams = [1200,1400]
    result = ['lacam_only/','lacam-lns1/','lacam-lns1-lns2/']

    for c in commits:
        for agent in teams:
            for d in delay:
                if d == 0.01:
                    d = '0.010'
                if d == 0.05:
                    d = '0.050'
                input_file = './lifelong_benchmark/warehouse-s/agent-' + str(agent) + '_scen-delay'+delay_model+'-' + str(d) + '-1.json'
                
                if po == 1:
                    continue
                    output_json = exp + 'warehouse-s/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
                    output_log = exp + 'warehouse-s/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
                    output_cout = exp + 'warehouse-s/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

                    cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(planner) + " --delayPolicy " + str(po) + " --delaySimulateAll true" + " >" + output_cout
                    p = subprocess.Popen(cmd,shell=True)
                    p.wait()
                    print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate all finished')
                if po == 2:
                    output_json = exp + 'warehouse-s/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
                    output_log = exp + 'warehouse-s/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
                    output_cout = exp + 'warehouse-s/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

                    cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(2) + " --delayPolicy " + str(po) + " --delaySimulateAll false" + " >" + output_cout
                    p = subprocess.Popen(cmd,shell=True)
                    p.wait()
                    print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate window finished')

                output_json = exp + 'warehouse-s/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
                output_log = exp + 'warehouse-s/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
                output_cout = exp + 'warehouse-s/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

                cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(planner) + " --delayPolicy " + str(po) + " --delaySimulateAll false" + " >" + output_cout
                p = subprocess.Popen(cmd,shell=True)
                p.wait()
                print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate window finished')


    # # #run game
    # # #mapf_planner = [1,2,3]
    # teams = [i for i in range(200,1401,200)]
    # result = ['lacam_only/','lacam-lns1/','lacam-lns1-lns2/']

    # #for planner in mapf_planner:
    # for c in commits:
    #     for agent in teams:
    #         for d in delay:
    #             if d == 0.01:
    #                 d = '0.010'
    #             if d == 0.05:
    #                 d = '0.050'
    #             input_file = './lifelong_benchmark/game/agent-' + str(agent) + '_scen-delay'+delay_model+'-' + str(d) + '-1.json'
    #             output_json = exp + 'game/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
    #             output_log = exp + 'game/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
    #             output_cout = exp + 'game/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

    #             cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(planner) + " --delayPolicy " + str(po) + " --delaySimulateAll true" + " >" + output_cout
    #             p = subprocess.Popen(cmd,shell=True)
    #             p.wait()
    #             print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate all finished')

    #             output_json = exp + 'game/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
    #             output_log = exp + 'game/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
    #             output_cout = exp + 'game/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

    #             cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(planner) + " --delayPolicy " + str(po) + " --delaySimulateAll false" + " >" + output_cout
    #             p = subprocess.Popen(cmd,shell=True)
    #             p.wait()
    #             print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate window finished')


    # # #run city
    # # #mapf_planner = [1,2,3]
    # teams = [i for i in range(1000,1001,200)]
    # teams = [i for i in range(500,3501,500)]
    # #teams = [2500,3000,3500]
    # result = ['lacam_only/','lacam-lns1/','lacam-lns1-lns2/']

    # #for planner in mapf_planner:
    # for c in commits:
    #     for agent in teams:
    #         for d in delay:
    #             if d == 0.01:
    #                 d = '0.010'
    #             if d == 0.05:
    #                 d = '0.050'
    #             input_file = './lifelong_benchmark/city/agent-' + str(agent) + '_scen-delay'+delay_model+'-' + str(d) + '-1.json'
    #             output_json = exp + 'city/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
    #             output_log = exp + 'city/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
    #             output_cout = exp + 'city/' + delay_policy[po-1]+'-all/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

    #             cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(planner) + " --delayPolicy " + str(po) + " --delaySimulateAll true" + " >" + output_cout
    #             p = subprocess.Popen(cmd,shell=True)
    #             p.wait()
    #             print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate all finished')

    #             output_json = exp + 'city/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/result-' + str(agent) +'agents-delay-'+ str(d) +'.json'
    #             output_log = exp + 'city/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/log-' + str(agent)+'agents-delay-'+ str(d) +'.txt'
    #             output_cout = exp + 'city/' + delay_policy[po-1]+'-window/' + 'commit' + str(c) + '/cout-' + str(agent)+'agents-delay-'+ str(d) +'.txt'

    #             cmd = "./build/lifelong --inputFile " + input_file + " -o " + output_json + " --simulationTime 1000 --planTimeLimit " + str(c) + " --logFile " + output_log + " -c " + str(c) + " --mapfPlanner " + str(planner) + " --delayPolicy " + str(po) + " --delaySimulateAll false" + " >" + output_cout
    #             p = subprocess.Popen(cmd,shell=True)
    #             p.wait()
    #             print("run for policy",delay_policy[po-1],'delay',d,"agent",agent,' simulate window finished')