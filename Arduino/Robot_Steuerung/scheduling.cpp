
#include "Arduino.h"
#include "scheduling.h"

Scheduler::Scheduler(){

}

void Scheduler::deleteTask(String func_name){
  int index = -1;
  for(int i = 0; i<pTaskSize; i++){
    if(pseudoTask_array[i].name.equals(func_name)){
      index = i;
      break;
    }
  }
  
  Serial.println("deleting");
  Serial.flush();
  
  if(index >= 0){
    TaskFunction * new_pseudoTask_array = new TaskFunction[pTaskSize-1]();
    
    for(int i = 0; i<pTaskSize; i++){
      if(i != index){
        if(i < index)
          new_pseudoTask_array[i] = pseudoTask_array[i];
        else
          new_pseudoTask_array[i-1] = pseudoTask_array[i];
      } else {
		delete (pseudoTask_array+i);
		resetFlag = true;
      }
    }
    
    old_pseudoTask_array = pseudoTask_array;
    pseudoTask_array = new_pseudoTask_array;
    pTaskSize = pTaskSize - 1 ;
    Serial.println("finished");Serial.flush();
  }
}

/**
 * Higher Priority Value means function will be preferred
 */
PriorityFunction * Scheduler::getPriorityArray(){
    PriorityFunction * priority_array = 0;

  for(int i = 0; i<pTaskSize; i++){
    if(pseudoTask_array[i].next_tick <= millis()){

      pseudoTask_array[i].next_tick += pseudoTask_array[i].tick_duration;
            
      if(priority_array == 0){
        priority_array = new PriorityFunction();
        priority_array->t = &pseudoTask_array[i];
      }
      else
      {
        PriorityFunction * current = priority_array;
        bool least_prio = false;
        //Highest Prio abfangen
        if(pseudoTask_array[i].priority >= current->t->priority){
          current = new PriorityFunction();
          current->t = &pseudoTask_array[i];
          current->next = priority_array;
          priority_array->last = current;
          priority_array = current;
          continue;
        }
        //Stelle suchen
        while(current->t->priority > pseudoTask_array[i].priority){
          if(current->next == 0){
            least_prio = true;
            break;
          }
          current = current->next;
        }

        //Einordnen
        if(least_prio) {
          PriorityFunction * addition = new PriorityFunction();
          addition->t = &pseudoTask_array[i];
          addition->last = current;
          current->next = addition;
        } else {
          PriorityFunction * addition = new PriorityFunction();
          addition->t = &pseudoTask_array[i];
          addition->next = current;
          addition->last = current->last;
          current->last = addition;
          addition->last->next = addition;
        }
        
      }
    }
  }
  return priority_array;
}

void Scheduler::execute(){
  priority_array = getPriorityArray();

  if(priority_array != 0){
  PriorityFunction * current = priority_array;
    while(current != 0){
    if(resetFlag){
      resetFlag = false;
      break;
    }
    int returnValue = (*(current->t->function))();
    current = current->next;
    }
    current = priority_array;
    PriorityFunction * todelete = 0;
    while(current != 0){
    todelete = current;
    current = current->next;
    delete todelete;
    }
  }
  
  if(old_pseudoTask_array != 0){
    delete[] old_pseudoTask_array;
    old_pseudoTask_array = 0;
  }
}

void Scheduler::addFunction(VFuncs func, String task_name, long tick_dur, long next_tick, int priority){
    TaskFunction newFunc;
    newFunc.function = func;
    newFunc.tick_duration = tick_dur;
    newFunc.priority = priority;
    newFunc.next_tick = next_tick;
    newFunc.name = task_name;

    addFunction(newFunc);
}

void Scheduler::addFunction(TaskFunction ptr){
     TaskFunction * new_pseudoTask_array = new TaskFunction[pTaskSize+1]();
    
    for(int i = 0; i<pTaskSize; i++){
      new_pseudoTask_array[i] = pseudoTask_array[i];
    }

    new_pseudoTask_array[pTaskSize] = ptr;

    old_pseudoTask_array = pseudoTask_array;
    pseudoTask_array = new_pseudoTask_array;
    pTaskSize = pTaskSize +1 ;
}


