#!/usr/bin/env python


class Method:
	def __init__(self):
		self.averageTime = []
		self.covariance = []
		self.maxTime = []
		self.minTime = []
		self.path = ""
		self.sucTime = []
		self.failTime = []
	
	def setPath(self, filePath):
		self.path = filePath

	def initialize(self):
		self.averageTime = []
		self.covariance = []
		self.sucTime = []
		self.failTime = []

	def doMath(self):
		print "Calculating data from file path: " + self.path
		self.initialize()
		self.loadInFile()
		self.averageTime.append( self.average(self.sucTime) )
		self.averageTime.append( self.average(self.failTime) )
		self.calcCovaraince()

	def loadInFile(self):
		f = open(self.path, "r")
		line = f.readline()
		self.maxTime = [0,0]
		self.minTime = [float('inf'), float('inf')]
		while(line):
			time = int(line[8:])
			if(line[0]=='s'):
				if time < self.minTime[0]:
					self.minTime[0] = time
				if time > self.maxTime[0]:
					self.maxTime[0] = time
				self.sucTime.append(time)
			else:
				if time < self.minTime[1]:
					self.minTime[1] = time
				if time > self.maxTime[1]:
					self.maxTime[1] = time
				self.failTime.append(int(line[8:]))
			line = f.readline()
		f.close()

	def average(self, list):
		if(len(list)):
			return sum(list)/len(list)

	def calcCovaraince(self):
		cov = 0
		for elem in self.sucTime:
			cov += (elem-self.averageTime[0])**2
		self.covariance.append(cov/(len(self.sucTime)-1))
		cov = 0
		for elem in self.failTime:
			cov += (elem-self.averageTime[1])**2
		self.covariance.append(cov/(len(self.failTime)-1))			

	def printResult(self):
		print "Success times: " , len(self.sucTime), "Failed times: ", len(self.failTime)
		print "Avr: ", self.averageTime, "Cov: ", self.covariance
		print "maxTime: " , self.maxTime, "minTime: " , self.minTime

if __name__ == "__main__":
	filePathFix = ["./obsFixed/overall/1.txt",
		    "./obsFixed/overall/2.txt",
		    "./obsFixed/overall/3.txt",
			"./obsFixed/overall/4.txt",
			"./obsFixed/overall/5.txt"]
	filePathRan = ["./obsRandom/overall/1.txt",
		    "./obsRandom/overall/2.txt",
		    "./obsRandom/overall/3.txt",
			"./obsRandom/overall/4.txt",
			"./obsRandom/overall/5.txt"]
	print "================================"
	a = Method()
	for i in {0,1,2,3,4}:	
		a.setPath(filePathRan[i])
		a.doMath()
		a.printResult()
		print "================================"
