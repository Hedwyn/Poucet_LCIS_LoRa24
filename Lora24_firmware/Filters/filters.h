#include <stdio.h>
#include <array>
#include <float.h>
#include <stdbool.h>

/** Error codes */
enum list_retcode
{
    OPE_SUCCESS = 1,
    LIST_TOO_SHORT,
    NULL_PTR
};

class LinkedElement
{
    public:
    LinkedElement(double value, LinkedElement* nextElement = NULL, LinkedElement *prevElement = NULL):value(value), nextElement(nextElement), prevElement(prevElement) {}
    ~LinkedElement();
    double value;
    LinkedElement *nextElement;
    LinkedElement *prevElement;
};

class LinkedList
{
    public:
    LinkedList();
    int getDepth();
    LinkedElement *head;
    LinkedElement *tail;
    list_retcode remove(int index);
    list_retcode remove(LinkedElement *element);
    list_retcode insert(LinkedElement *element, int index);
    void insert(LinkedElement *element);
    virtual void append(LinkedElement *element);
    void insertSorted(LinkedElement *element);
    double *get(int index, double *value);
};

class Fifo
{
    public:
    Fifo(const int length);
    const int length;
    LinkedElement** elements;
    void stack(LinkedElement* element); 
    virtual LinkedElement* unstack();
    int getHeight();

    private:
    int inIndex;
    int outIndex;
    bool isFull;
    
};

class MovingMedian: public LinkedList, public Fifo
{
    public:
    MovingMedian(const int length):LinkedList(), Fifo(length) {}
    double compute();
    double compute(int meanLength);
    void append(double value);
    void accumulate(double value);
    LinkedElement* unstack();
    Fifo *accumulator;
};