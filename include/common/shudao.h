#ifndef __DNET_SHUDAO_H__
#define __DNET_SHUDAO_H__

#include <string>	
#include <dnet.h>
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__GNUG__)
#	define SD_EXPORT __attribute__ ((visibility("default")))
#include <pthread.h>
#else
#	define SD_EXPORT __declspec(dllexport)
#endif
	class SD_EXPORT dnet_icd_t
	{
	public:
		virtual void* get_send_data(int &send_size) =0;
		virtual void  set_recv_data(void* recv_data, int recv_size)=0;
		virtual void* get_header(int &header_size) = 0;
		virtual void* set_header(void* header, int header_size)=0; 
	};
	
//		class SD_EXPORT dnet_rwbuf_t
//		{
//		public:
//			void  set_pack(dnet_msg_t* header, dnet_msg_t* pack_msg);
//			int   get_pack(dnet_msg_t &header, dnet_msg_t &pack_msg);
//		private:
//			dnet_mutex_t locker;
//			std::basic_string<u_char>  mem_head;
//			std::basic_string<u_char>  mem_pack;
//		};
	class SD_EXPORT dnet_rwbuf_t
	{
	public:
		void  set_pack(dnet_msg_t* header, dnet_msg_t* pack_msg);
		int  get_pack(dnet_msg_t &header, dnet_msg_t &pack_msg);
	private:
		dnet_mutex_t locker;
		dnet_msg_t  header;
		dnet_msg_t  pack_msg;
	};
	class SD_EXPORT dnet_ringbuf_t
	{
	public:
		dnet_ringbuf_t(u_int buf_num);
		~dnet_ringbuf_t();
	public:
		void set_back(dnet_msg_t* header, dnet_msg_t* pack_msg);
		void set_front(dnet_msg_t* header, dnet_msg_t* pack_msg);
		void set_at(dnet_msg_t* header, dnet_msg_t* pack_msg, u_int index);
		void get_back(dnet_msg_t& header, dnet_msg_t &pack_msg);
		void get_front(dnet_msg_t& header, dnet_msg_t &pack_msg);
		void get_at(dnet_msg_t& header, dnet_msg_t &pack_msg, u_int index);
	private:
		void* mem_buf;	
	};
	
class SD_EXPORT dnet_task_t
	{
	public:
		dnet_task_t();
		~dnet_task_t();
	public:
		virtual void task0_step();
		virtual void task1_step();
		virtual void task2_step();
		virtual void task3_step();
		virtual void task4_step();
		
		virtual void task0_loop();
		virtual void task1_loop();
		virtual void task2_loop();
		virtual void task3_loop();
		virtual void task4_loop();
	public:
		void start(u_int thread_num = 1);
		void stop();
		void priority(int priority, u_int task_index);
		void cpumask(int cpumask, u_int task_index);
		static int cpucores();
		int thread_num;
		
	protected:
		bool is_stopping[5];
		bool is_stopped[5];
#if defined(__GNUC__) || defined(__GNUG__)
		pthread_t descriptor[5];
#else
		void* descriptor[5];
#endif
	};
	
	
	#undef SD_EXPORT
	

	#ifdef __cplusplus
	}
#endif

#endif
