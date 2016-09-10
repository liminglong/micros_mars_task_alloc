/*************************************************************************************************************************
 * Author:      Minglong Li
 * Affiliation: State Key Laboratory of High Performance Computing (HPCL)
 *              College of Computer, National University of Defense Technology
 * Email:       minglong_l@163.com
 * Created on:  Sep. 7th, 2016               
 **************************************************************************************************************************/

#ifndef RING_QUEUE_H_
#define RING_QUEUE_H_
 
namespace micros_mars_task_alloc
{
    template<typename T>
    class RingQueue
    {
    public:
    　　　　RingQueue(int size): size_(size){};
        ~RingQueue(){};
        void add(T & value);
        void del(T & value);
        void size();
        //重定义操作符[],使得它像访问正常数组一样访问这个环形数组。
    
    private:
        int size_;
    }
 
}//namespace micros_mars_task_alloc

#endif
