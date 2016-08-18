import numpy as np
import math, sys




def perp( a ):
	b = np.empty_like(a)
	b[0] = -a[1]
	b[1] = a[0]
	return b

def seg_intersect(a1, a2, b1, b2) :
	da = a2-a1
	db = b2-b1
	dp = a1-b1
	dap = perp(da)
	denom = np.dot( dap, db)
	num = np.dot( dap, dp )
	if abs(denom.astype(float)) < 0.0000001:
		raise RuntimeWarning
	ret = (num / denom.astype(float))*db + b1
	for p in ret:
		if np.isnan(p) or np.isinf(p):
			raise RuntimeWarning
	return ret


def get_lines():
	text_lines = open(sys.argv[1]).readlines()
	lines = []
	for text_line in text_lines:
		coords = [ float(x) for x in text_line.split(',') ]
		p1 = np.array([coords[0], coords[1]])
		p2 = np.array([coords[2], coords[3]])
		lines.append([p1, p2])
	return lines

def ccw(A,B,C):
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

lines = get_lines()

nodes = []
edges = []



for i1, l1 in enumerate(lines):
	# For each line, add the endpoints to the 'nodes' dictionary
	for inew_node, new_node in enumerate(l1):
		do = True
		#print inew_node, new_node
		for iknown_node, known_node in enumerate(nodes):
			if not np.any(new_node-known_node):
				do = False
				if inew_node == 0:
					index1p1 = iknown_node
				else:
					index1p2 = iknown_node
				break
		#print 'do?', do
		if do:
			#print 'b1', nodes
			nodes.append(new_node)
			#print 'b2', nodes
			if inew_node == 0:
				index1p1 = len(nodes)-1
			else:
				index1p2 = len(nodes)-1
			
	edges.append([index1p1, index1p2])

	for i2, l2 in enumerate(lines):
		if i1 >= i2:
			continue
		if np.array_equal(l1, l2):
			continue

		#print 'l2', l2
		# Add the endpoints of possible new lines to the 'nodes' dictionary
		for inew_node, new_node in enumerate(l2):
			#print 'new_node', new_node
			do = True
			for iknown_node, known_node in enumerate(nodes):
				if not np.any(new_node-known_node):
					do = False
					if inew_node == 0:
						index2p1 = iknown_node
					else:
						index2p2 = iknown_node
					break
			#print 'do?', do
			if do:
				#print 'c1', nodes
				nodes.append(new_node)
				#print 'c2', nodes
				if inew_node == 0:
					index2p1 = len(nodes)-1
				else:
					index2p2 = len(nodes)-1

		try:
			if intersect(l1[0], l1[1], l2[0], l2[1]):
				r = seg_intersect(l1[0], l1[1], l2[0], l2[1])
				#print '<<<intersection****'
				#print l1[0], l1[1], l2[0], l2[1]
				#print r
				#print '****intersection>>>'
				ir = len(nodes)
				nodes.append(r)
				edges.append([index1p1, ir])
				edges.append([index1p2, ir])
				edges.append([index2p1, ir])
				edges.append([index2p2, ir])
		except RuntimeWarning:
			pass
	#print l
	


print len(nodes)
for n in nodes:
	print str(float(n[0]))+'#'+str(float(n[1]))
for e in edges:
	print str(e[0])+'#'+str(e[1])



