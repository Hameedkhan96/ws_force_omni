from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

def load(arg,_file):
	if arg   == 'Fd':    k=0
	elif arg == 'F':     k=1
	elif arg == 'e':     k=2
	elif arg == 'xd':     k=3
	else:
		print ('invalid argument')
		return 0
	col = []
	with open(_file+'.txt','r') as files:
		#open('/home/santos-lap/Documents/Research/ICCC_2024/'+_file+'.txt','r') as files:
		data = files.readlines()
		for line in data:
			col.append(float(line.rsplit(',')[k]))
	return col
	
font_name = 'Liberation Serif'
titles = 0
legends = 0
title_size = 28
label_size = 28
tick_size = 28
legend_size = 28
fd_color = '#000000'
fi_color = '#000000'
fh_color = '#000000'
file_name = 'Force'
Fd = load('Fd',file_name)
F  = load('F',file_name)

t = np.linspace(0,int(len(F)/50.),len(F))

#F  = np.zeros(len(t))

	
plt.figure(1)
if titles==1:
	plt.suptitle('Applied Force',fontsize=title_size,fontname=font_name)
#plt.subplot(2,3,1)
plt.plot(t,Fd,color=fd_color,linestyle='dashed',linewidth=3)
#plt.plot(t,F ,color=fh_color,linestyle='dashed',linewidth=3)
plt.plot(t,F ,color=fh_color,linewidth=3)
#plt.grid()
plt.xlabel('[min]',fontsize=label_size,fontname=font_name)
plt.ylabel('[N]',fontsize=label_size,fontname=font_name)
plt.yticks(fontsize=tick_size,fontname=font_name)
plt.xticks(fontsize=tick_size,fontname=font_name)
if legends==1:
	plt.legend([r'$F$',r'$F^d$'],prop={'family':font_name, 'size':legend_size})
plt.subplots_adjust(left=0.145, bottom=0.18, right=0.99, top=0.98, wspace=None, hspace=None)

plt.show()
