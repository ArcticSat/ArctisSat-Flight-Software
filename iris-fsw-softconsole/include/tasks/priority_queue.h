#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// File Description:
//  Template header file for C / C++ projects. Unused sections can be deleted.
//
// History
// 2019-01-13 by Tamkin Rahman
// - Created.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// INCLUDES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// DEFINITIONS AND MACROS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// ENUMS AND ENUM TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// STRUCTS AND STRUCT TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct node {
	unsigned long priority; // Lower values indicate higher priority
	struct node* next;
	void* data;
} Node;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// TYPEDEFS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// CONSTANTS
//-------------------------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Create new node for linked list with given data pointer and given priority.
//
// Parameters:
//	data - the data (stored as pointer)
//  priority - the priority of the node, lower number is higher priority. (i.e., 0 is highest priority)
// Returns:
//  Pointer to a new node structure
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
Node* newNode(void* data, unsigned long priority);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Return the highest priority value without removing it from the queue.
//
// Parameters:
//	head - a pointer to a pointer containing head
// Returns:
//  The data value of the highest priority node without removing it from the queue.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void* peek(Node** head);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Removes the node with highest priority from the queue.
//
// Parameters:
//	head - a pointer to a pointer containing head
// Returns:
//  VOID
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void pop(Node** head);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Checks if the list is empty. 1 if empty, 0 otherwise.
//
// Parameters:
//	head - a pointer to a pointer containing head
// Returns:
//  int - 1 if empty, 0 otherwise
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
int isEmpty(Node** head);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Pushes a node with data [d] and priority [p] onto priority queue [head].
//
// Parameters:
//	head - a pointer to a pointer containing head
//	d	 - a pointer to the data to store
//	p	 - priority of node (lower priority value will be higher in queue, i.e. 0 is highest priority)
// Returns:
//  The data value of the highest priority node without removing it from the queue.
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void push(Node** head, void* d, unsigned long p);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Test the functionality of the priority queue data structure.
//
// Returns:
//  Void
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void vTaskTest_Priority_Queue(void *pvParameters);

#endif // PRIORITY_QUEUE_H
