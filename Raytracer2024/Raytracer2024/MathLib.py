from math import pi, sin, cos, isclose

def barycentricCoords(A, B, C, P):
	areaPCB = abs((P[0]*C[1] + C[0]*B[1] + B[0]*P[1]) - 
				  (P[1]*C[0] + C[1]*B[0] + B[1]*P[0]))

	areaACP = abs((A[0]*C[1] + C[0]*P[1] + P[0]*A[1]) - 
				  (A[1]*C[0] + C[1]*P[0] + P[1]*A[0]))

	areaABP = abs((A[0]*B[1] + B[0]*P[1] + P[0]*A[1]) - 
				  (A[1]*B[0] + B[1]*P[0] + P[1]*A[0]))

	areaABC = abs((A[0]*B[1] + B[0]*C[1] + C[0]*A[1]) - 
				  (A[1]*B[0] + B[1]*C[0] + C[1]*A[0]))

	if areaABC == 0:
		return None

	u = areaPCB / areaABC
	v = areaACP / areaABC
	w = areaABP / areaABC

	if 0 <= u <= 1 and 0 <= v <= 1 and 0 <= w <= 1:
		return (u, v, w)
	else:
		return None

def TranslationMatrix(x, y, z):
	return [[1, 0, 0, x],
			[0, 1, 0, y],
			[0, 0, 1, z],
			[0, 0, 0, 1]]

def ScaleMatrix(x, y, z):
	return [[x, 0, 0, 0],
			[0, y, 0, 0],
			[0, 0, z, 0],
			[0, 0, 0, 1]]

def RotationMatrix(pitch, yaw, roll):
	pitch *= pi / 180
	yaw *= pi / 180
	roll *= pi / 180
	
	pitchMat = [[1, 0, 0, 0],
				[0, cos(pitch), -sin(pitch), 0],
				[0, sin(pitch), cos(pitch), 0],
				[0, 0, 0, 1]]

	yawMat = [[cos(yaw), 0, sin(yaw), 0],
			  [0, 1, 0, 0],
			  [-sin(yaw), 0, cos(yaw), 0],
			  [0, 0, 0, 1]]

	rollMat = [[cos(roll), -sin(roll), 0, 0],
			   [sin(roll), cos(roll), 0, 0],
			   [0, 0, 1, 0],
			   [0, 0, 0, 1]]

	return multiplyMatrices(multiplyMatrices(pitchMat, yawMat), rollMat)

def reflectVector(normal, direction):
    reflect = 2 * dot(normal, direction)
    reflect = multiply(reflect, normal)
    reflect = subtract(reflect, direction)
    reflect = normalize(reflect)
    return reflect

def dot(v1, v2):
    return sum(a * b for a, b in zip(v1, v2))

def normalize(v):
    length = magnitude(v)
    return [component / length for component in v] if length > 0 else v

def magnitude(v):
    return sum(component ** 2 for component in v) ** 0.5

def subtract(v1, v2):
    return [a - b for a, b in zip(v1, v2)]

def multiply(scalar, vector):
    return [scalar * component for component in vector]


def dotProduct(v1, v2):
	return sum(v1[i] * v2[i] for i in range(len(v1)))

def vectorMagnitude(v):
	return sum(x ** 2 for x in v) ** 0.5

def multiplyMatrices(m1, m2):
	result = [[0 for _ in range(len(m2[0]))] for _ in range(len(m1))]
	for i in range(len(m1)):
		for j in range(len(m2[0])):
			for k in range(len(m2)):
				result[i][j] += m1[i][k] * m2[k][j]
	return result
