%Example on how to add java class and call functions from matlab

%matlab can execute java classes that are added to at least to the dynamic
%path _--- this an absolute path for now
%javaclasspath('-dynamic')
javaaddpath('D:\GIT\Work\efop\java\bin')
javaclasspath('-dynamic')

myclass = efop_java.HelloJava()
methodsview efop_java.HelloJava
numberFromMyClass = myclass.getNumber()
myclass.setNumber(5)
numberFromMyClass = myclass.getNumber()