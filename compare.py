import matplotlib.pyplot as plt
import matplotlib.image as mpimg

fig, ax = plt.subplots()
#random 100,200,300,400
teams = [100,200,300,400]
# lacam = [3.569,4.944,5.258,5.250]
# lacam_p05 = [1.418,0.590,0.447,0.392]
# lacam_p1  = [1.051,0.661,0.542,0.417]
# lacam_p15  = [1.444,0.824,0.344,0.373]
# lacam_p2  = [1.330,0.645,0.398,0.255]
# plt.plot(teams, lacam,'X-', label = "p = 0", markersize=10)
# plt.plot(teams, lacam_p05,'P-', label = "p = 0.05", markersize=10)
# plt.plot(teams, lacam_p1,'<-', label = "p = 0.1", markersize=10)
# plt.plot(teams, lacam_p15,'>-', label = "p = 0.15", markersize=10)
# plt.plot(teams, lacam_p2,'o-', label = "p = 0.2", markersize=10)

# replanall = [3.615,6.814,9.374,9.523]
# replanall_p05 = [3.618,4.612,2.062,0.734]
# replanall_p1  = [3.362, 4.245,2.942,0.948]
# replanall_p15  = [3.132,3.983,2.282,0.791]
# replanall_p2  = [2.895,3.551,2.106,0.735]
# plt.plot(teams, replanall,'X-', label = "p = 0", markersize=10)
# plt.plot(teams, replanall_p05,'P-', label = "p = 0.05", markersize=10)
# plt.plot(teams, replanall_p1,'<-', label = "p = 0.1", markersize=10)
# plt.plot(teams, replanall_p15,'>-', label = "p = 0.15", markersize=10)
# plt.plot(teams, replanall_p2,'o-', label = "p = 0.2", markersize=10)

replanaff = [4.302,7.970,10.663,11.204]
replanaff_p05 = [3.670,5.131,4.405,3.414]
replanaff_p1  = [3.426,4.582,3.957,3.053]
replanaff_p15  = [3.155,4.244,3.376,2.705]
replanaff_p2  = [2.941,3.740,3.182,2.247]
plt.plot(teams, replanaff,'X-', label = "p = 0", markersize=10)
plt.plot(teams, replanaff_p05,'P-', label = "p = 0.05", markersize=10)
plt.plot(teams, replanaff_p1,'<-', label = "p = 0.1", markersize=10)
plt.plot(teams, replanaff_p15,'>-', label = "p = 0.15", markersize=10)
plt.plot(teams, replanaff_p2,'o-', label = "p = 0.2", markersize=10)

# pic = "pic_map/random-32-32-20.png"
# result = "throught-random-rhcr.pdf"

# #warehouse 200,400,600,800,1000
# teams = [200,400,600,800,1000,1200,1400]
# lacam = [1.816,3.090,3.893,4.356,4.602,4.387,4.124]
# rhcr = [2.225,4.313,3.324,1.680,1.648,1.563,1.317]
# oursl = [2.092,4.252,6.138,7.891,9.526,7.535,5.982] #commit=20
# oursf = [2.335,4.745,6.824,8.891,11.005,12.4,0.79]
# pic = 'pic_map/warehouse-10-20-10-2-1.png'
# result = "throught-warehouse-rhcr.pdf"

# #game 200,400,600,800,1000
# teams = [200,400,600,800,1000,1200,1400]
# lacam = [1.734,2.570,3.180,3.385,3.542,3.777,3.88] 
# rhcr = [1.842,0.716,0.581,0.311,0.282,0.374,0.293]
# oursl = [1.701,3.314,4.552,4.624,4.784,4.791,4.601] #commit = 20
# oursf = [1.857,3.644,5.006,5.319,5.176,4.102,3.337]
# pic = "pic_map/ht_mansion_n_crop.png"
# result = "throught-game-rhcr.pdf"

# #city 200,400,600,800,1000

# teams = [500,1000,1500,2000,2500,3000,3500]
# lacam = [2.357,4.609,6.689,8.860,10.905,12.337,13.435]
# rhcr = [2.390,4.746,7.179,1.536,1.339,1.431,1.114]
# oursl = [2.297,4.581,6.873,9.062,11.424,13.320,15.417]#commit = 20, missing 500 results
# #oursf = [2.409,4.776,7.206,9.528]
# oursf = [2.393,4.754,7.169,9.476,11.953,14.094,16.655]#commit = 3
# pic = 'pic_map/Paris_1_256.png'
# result = "throught-city-rhcr.pdf"

# fig, ax = plt.subplots()


# plt.plot(teams, lacam,'X-', label = "p = 0", markersize=10)
# plt.plot(teams, lacam_p05,'P-', label = "p = 0.05", markersize=10)
# plt.plot(teams, lacam_p1,'<-', label = "p = 0.1", markersize=10)
# plt.plot(teams, lacam_p15,'>-', label = "p = 0.15", markersize=10)
# plt.plot(teams, lacam_p2,'o-', label = "p = 0.2", markersize=10)

plt.xticks(teams,fontsize=20)
plt.yticks(fontsize=20)
plt.grid(axis='y',ls='--',alpha=0.8)
#plt.ylim((0,13))
#plt.legend(fontsize=20,frameon=False, loc = 'upper center',bbox_to_anchor=(0.5,-0.05))
plt.legend(fontsize=20, frameon=False)

plt.title('Replan Affect',fontsize=20)
plt.ylabel('Throughput',fontsize=30)
plt.xlabel('Number of Agents',fontsize=20)
# im = mpimg.imread(pic)
# ax2 = fig.add_axes([-0.01, 0.6, 0.3, 0.3], anchor='SE', zorder=3)
# ax2.imshow(im)
# ax2.axis('off')





plt.tight_layout()
#plt.savefig(result,bbox_inches='tight')
plt.show()