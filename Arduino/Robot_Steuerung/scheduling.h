#ifndef SCHEDULING_H_
#define SCHEDULING_H_

#include "Arduino.h"

typedef int (*VFuncs)(void);

typedef struct{
  VFuncs function;
  long next_tick;
  long tick_duration;
  long priority = 0;
  String name;
} TaskFunction;

typedef struct PriorityFunction{
  TaskFunction * t = 0;
  PriorityFunction * last = 0;
  PriorityFunction * next = 0;
};

class Scheduler{
public:
  Scheduler();
  TaskFunction * pseudoTask_array;
  TaskFunction * old_pseudoTask_array = 0;
  int pTaskSize = 0;
  bool resetFlag = false;
  PriorityFunction * priority_array;

  void deleteTask(String);
  PriorityFunction * getPriorityArray();
  void execute();
  void addFunction(VFuncs, String, long, long, int);
  void addFunction(TaskFunction);
};

#endif

