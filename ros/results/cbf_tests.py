# /usr/bin/env python3.3 or higher

import jpy
import numpy as np
from matplotlib import pyplot as plt

# Create a new JVM with the SL library on its classpath
jpy.create_jvm(['-Xmx512M', '-Djava.class.path=/home/malte/catkin_ws/src/knowledge_aggregation/subjective_logic/build/libs/subjective-logic-java-library-0.1.0.jar'])

# Get a reference of the SubjectiveOpinion Java class through jpy bridge
SubjectiveOpinion = jpy.get_type('de.tum.i4.subjectivelogic.SubjectiveOpinion')

# Create two subjective opinions
so_vacuous = SubjectiveOpinion(0.0, 0.7, 0.3, 0.50)
so_task = SubjectiveOpinion(0.7, 0.0, 0.3, 0.50)

ArrayList = jpy.get_type('java.util.ArrayList')
solist = ArrayList()


repetition_list = []
belief_list = []
disbelief_list = []
uncert_list = []
exp_list = []

belief_diff = []
disbelief_diff = []
uncert_diff = []

for i in np.arange(100,10000,100):
    repetition_list.append(i)
    
    prev_opinion = SubjectiveOpinion(0.0, 0.0, 1.0, 0.50)
    for j in range(i):
        # Add the subjective opinions to the array list
        solist.add(so_vacuous)
        solist.add(prev_opinion)

        # Perform Cumulative Fusion of the opinions in the list
        cbf = SubjectiveOpinion.cumulativeCollectionFuse(solist)
        prev_opinion = cbf
        solist.clear()

    # no add back the task opinions
    # equal times
    '''
    for j in range(i):
        # Add the subjective opinions to the array list
        solist.add(so_task)
        solist.add(prev_opinion)

        # Perform Cumulative Fusion of the opinions in the list
        cbf = SubjectiveOpinion.cumulativeCollectionFuse(solist)
        prev_opinion = cbf
        solist.clear()
    belief_list.append(prev_opinion.getBelief())
    disbelief_list.append(prev_opinion.getDisbelief())
    uncert_list.append(prev_opinion.getUncertainty())
    exp_list.append(prev_opinion.getBelief() + prev_opinion.getUncertainty() * prev_opinion.getBaseRate())
    '''
    # just a single time and calc difference
    solist.add(so_task)
    solist.add(prev_opinion)

    # Perform Cumulative Fusion of the opinions in the list
    cbf = SubjectiveOpinion.cumulativeCollectionFuse(solist)
    
    belief_diff.append(cbf.getBelief() - prev_opinion.getBelief())
    disbelief_diff.append(cbf.getDisbelief() - prev_opinion.getDisbelief())
    uncert_diff.append(cbf.getUncertainty() - prev_opinion.getUncertainty())

    solist.clear()



######## Testing if uncertainty can decrease #########

dec_solist = ArrayList()

print(so_vacuous.toString())
dec_solist.add(so_vacuous)
dec_solist.add(so_vacuous)

combined_so = SubjectiveOpinion.cumulativeCollectionFuse(dec_solist)
print(combined_so.toString())

dec_solist.clear()
dec_solist.add(combined_so)
dec_solist.add(so_task)

combined_so = SubjectiveOpinion.cumulativeCollectionFuse(dec_solist)
print(combined_so.toString())

dec_solist.clear()
dec_solist.add(combined_so)
dec_solist.add(so_task)

combined_so = SubjectiveOpinion.cumulativeCollectionFuse(dec_solist)
print(combined_so.toString())
######### Fractional Plot ##############

frac_belief = []
frac_disbelief = []
frac_uncert = []
frac_exp = []
frac_repetition = []

frac_solist = ArrayList()

# add 10,000 vacuous opinions
flag = True
prev_opinion = SubjectiveOpinion(0.0, 0.0, 1.0, 0.50) # empty
saved_opinion = SubjectiveOpinion(0.0, 0.0, 1.0, 0.50)
for i in range(1,10001):
    
    # Add the subjective opinions to the array list
    frac_solist.add(so_vacuous)
    frac_solist.add(prev_opinion)

    # Perform Cumulative Fusion of the opinions in the list
    cbf = SubjectiveOpinion.cumulativeCollectionFuse(frac_solist)
    prev_opinion = cbf

    if flag == True and cbf.getDisbelief()>= 0.99:
        print('Number of additions are: ', i)
        saved_opinion = cbf
        flag = False

    frac_solist.clear()

####### brief interupting test, check how often need to add task opinion to recover for saved state
print('START ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())
for i in range(1,100):

    # Add the subjective opinions to the array list
    frac_solist.add(so_task)
    frac_solist.add(saved_opinion)

    # Perform Cumulative Fusion of the opinions in the list
    cbf = SubjectiveOpinion.cumulativeCollectionFuse(frac_solist)
    saved_opinion = cbf

    if flag == False and saved_opinion.getBelief()>= 0.4:
        print('Number of additions are: ', i)
        flag = True
        print('Middle ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())

    frac_solist.clear()
print('End ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())

# get initial values
frac_repetition.append(0)
frac_belief.append(prev_opinion.getBelief())
frac_disbelief.append(prev_opinion.getDisbelief())
frac_uncert.append(prev_opinion.getUncertainty())
frac_exp.append(prev_opinion.getBelief() + prev_opinion.getUncertainty() * prev_opinion.getBaseRate())

# now slowly add back task opinions and note difference    
for i in range(1,10001):
    frac_repetition.append(float(i) / 10001)

    # Add the subjective opinions to the array list
    frac_solist.add(so_task)
    frac_solist.add(prev_opinion)

    # Perform Cumulative Fusion of the opinions in the list
    cbf = SubjectiveOpinion.cumulativeCollectionFuse(frac_solist)
    prev_opinion = cbf
    
    frac_solist.clear()
    
    frac_belief.append(prev_opinion.getBelief())
    frac_disbelief.append(prev_opinion.getDisbelief())
    frac_uncert.append(prev_opinion.getUncertainty())
    frac_exp.append(prev_opinion.getBelief() + prev_opinion.getUncertainty() * prev_opinion.getBaseRate())


### Plotting

fig_num = 1
'''
fig = plt.figure(fig_num)
plt.plot(repetition_list, belief_list, label='Belief')
plt.plot(repetition_list, disbelief_list, label='Disbelief')
#plt.plot(repetition_list, uncert_list, label='Uncertainty')
plt.plot(repetition_list, exp_list, label='Expected Value')
plt.legend()
plt.xlabel('Repetitions')
plt.ylabel('value')
plt.savefig('plots/testing_CBF.png')
fig_num += 1
'''

# plotting diff
'''
fig = plt.figure(fig_num)
plt.plot(repetition_list, belief_diff, label='Belief Diff')
plt.plot(repetition_list, disbelief_diff, label='Disbelief Diff')
plt.plot(repetition_list, uncert_diff, label='Uncertainty Diff')
#plt.plot(repetition_list, exp_list, label='Expected Value Diff')
plt.legend()
plt.xlabel('Repetitions')
plt.ylabel('Value Diff (new - previous)')
plt.savefig('plots/testing_CBF.png')
#print ("CBF: " + cbf.toString())
#print(cbf.getBelief(), " ", cbf.getDisbelief(), " ", cbf.getUncertainty())
fig_num += 1
'''

# plotting frac
fig = plt.figure(fig_num)
plt.plot(frac_repetition, frac_belief, label='Belief')
#plt.plot(frac_repetition, frac_disbelief, label='Disbelief')
#plt.plot(frac_repetition, frac_uncert, label='Uncertainty')
#plt.plot(frac_repetition, frac_exp, label='Expected Value')
plt.legend()
plt.xlabel('Fractional Repetitions')
plt.ylabel('Value')
plt.savefig('plots/testing_CBF_frac.png')

fig_num += 1

# plotting frac difference

fig = plt.figure(fig_num)
plt.plot(frac_repetition[1:], np.array(frac_belief[1:]) - np.array(frac_belief[:-1]), label='Belief Diff')
plt.plot(frac_repetition[1:], np.array(frac_disbelief[1:]) - np.array(frac_disbelief[:-1]), label='Disbelief Diff')
plt.plot(frac_repetition[1:], np.array(frac_uncert[1:]) - np.array(frac_uncert[:-1]), label='Uncertainty Diff')
plt.plot(frac_repetition[1:], np.array(frac_exp[1:]) - np.array(frac_exp[:-1]), label='Expected Value Diff')
plt.legend()
plt.xlabel('Fractional Repetitions')
plt.ylabel('Value')
plt.savefig('plots/testing_CBF_frac_diff.png')

fig_num += 1



###### Test recovery with very small uncertainty
flag = True
saved_opinion = SubjectiveOpinion(0.01 - 1e-9, 0.99, 1e-9, 0.50)
print('START 2 ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())
for i in range(1,3000):

    # Add the subjective opinions to the array list
    frac_solist.add(so_task)
    frac_solist.add(saved_opinion)

    # Perform Cumulative Fusion of the opinions in the list
    #cbf = SubjectiveOpinion.cumulativeCollectionFuse(frac_solist)
    cbf = SubjectiveOpinion.ccCollectionFuse(frac_solist)
    #cbf = SubjectiveOpinion.average(frac_solist)
    saved_opinion = cbf

    if flag == True and saved_opinion.getBelief()>= 0.7:
        print('Number of additions are: ', i)
        flag = False
    print('Middle ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())

    frac_solist.clear()
print('End ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())



####### Test if CCF or weighted can accumulate belief ###
'''
saved_opinion = SubjectiveOpinion(0.0, 0., 1, 0.50)
print('START Accumulate test ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())
for i in range(1,3000):

    # Add the subjective opinions to the array list
    frac_solist.add(so_task)
    frac_solist.add(saved_opinion)

    # Perform Cumulative Fusion of the opinions in the list
    #cbf = SubjectiveOpinion.cumulativeCollectionFuse(frac_solist)
    cbf = SubjectiveOpinion.ccCollectionFuse(frac_solist)
    #cbf = SubjectiveOpinion.average(frac_solist)
    saved_opinion = cbf

    print('Middle ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())

    frac_solist.clear()
print('End ', saved_opinion.getBelief(), " ", saved_opinion.getDisbelief(), " ", saved_opinion.getUncertainty())
'''

'''
frac_solist.add(SubjectiveOpinion(0.3,0.5,0.2,0.5))
frac_solist.add(SubjectiveOpinion(0.8,0.1,0.1,0.5))

cbf = SubjectiveOpinion.ccCollectionFuse(frac_solist)

print('Combine CCF; ', cbf.getBelief(), " ", cbf.getDisbelief(), " ", cbf.getUncertainty())


cbf = SubjectiveOpinion.cumulativeCollectionFuse(frac_solist)

print('Combine CBF; ', cbf.getBelief(), " ", cbf.getDisbelief(), " ", cbf.getUncertainty())

'''







