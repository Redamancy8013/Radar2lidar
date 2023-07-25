#ifndef __DNET_H_INCLUDED__
#define __DNET_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__GNUG__)
#	define DNET_EXPORT __attribute__ ((visibility("default")))
#include <pthread.h>
#else
#	define DNET_EXPORT __declspec(dllexport)
#endif

#ifndef __MDEBUG__
#define __MDEBUG__	/**������뷢�а�,��ע�͵���һ��<*/
#endif
#ifdef __MDEBUG__
#include "stdio.h"
#define sd_debug(format,...) printf(format,##__VA_ARGS__)
#else
#define sd_debug(format,...)
#endif

	/**DEBUG_LINE: ���Ҫ��ӡ���ļ����Ƽ��ļ����ڵ��к�,ע�������__DEBUG__<*/
	/**
	#ifndef __DEBUG_LINE__
	#define __DEBUG_LINE__
	#endif
	#ifdef __DEBUG_LINE__
	#include "stdio.h"
	#define sd_debug(format,...) printf("File: "__FILE__", Line: %05d: "format"\n",\
	__LINE__, ##__VA_ARGS__)
	#else
	#define sd_debug(format,...)
	#endif
	*/

#ifndef u_char
	typedef unsigned char u_char;
	typedef unsigned int  u_int;
	typedef unsigned short u_short;
#endif
#include <stdint.h>
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	* �ڴ����: ֧�ִ��ڴ涯̬����
	*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	typedef void(raw_free_fn)(void* data, void* arg);
	class DNET_EXPORT dnet_msg_t
	{
	public:
		dnet_msg_t();
		dnet_msg_t(void* data, size_t size);
		~dnet_msg_t();
	public:
		int re_init();
		int re_data(void* data, size_t size, raw_free_fn* free_fn = 0, void* arg = 0);

		int re_size(size_t size);
		int copy(dnet_msg_t* src);
		int size();
		void* data();
		void* raw();
		void clear();
		int more();

	private:
		void* msg;
	};
	
	
/**�׽������Ͷ���<*/
#define DNET_PAIR 0
#define DNET_PUB 1
#define DNET_SUB 2
#define DNET_UDP 3

	/**����/����ģʽ����<*/
#define DNET_BLOCK		0	//����ģʽ
#define DNET_NOBLOCK	1	//������ģʽ
#define DNET_MORE		2	//����/���ո���֡(������֡ģʽ)	

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	* ͨ����: ����-����,��-��
	*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	class DNET_EXPORT dnet_socket_t
	{
	public:
		dnet_socket_t();
		~dnet_socket_t();
	public:
		bool sock_init(int sock_type, int timeout);
		bool sock_addr(char* addr, bool isbind);
		int  send_msg(dnet_msg_t* snddata, int flag);
		int  recv_msg(dnet_msg_t& rcvdata, int flag);
		int  send_pack(void* header, int header_size, dnet_msg_t* snddata);
		int  recv_pack(void* header, int header_size, dnet_msg_t& rcvdata);
	private:
		void* socket;
	};
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	* ��ȷ��ʱ
	*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	class DNET_EXPORT dnet_timer_t
	{
	public:
		dnet_timer_t();
		~dnet_timer_t();
	public:
		void    start();
		void    reset();
		void	stop();
		int     elapse_us();
		int     elapse_ms();
		static uint64_t gettime_us();
	private:
		void* timer;
	};

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	* �߳�
	*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	class DNET_EXPORT dnet_thread_t
	{
	public:
		dnet_thread_t();
		~dnet_thread_t();
	public:
		virtual void step();
		bool start();
		void stop();
		virtual void loop();
		void priority(int priority);
		void cpumask(int cpumask);
		static int cpucores();
	protected:
		bool is_stopping;
		bool is_stopped;
#if defined(__GNUC__) || defined(__GNUG__)
		pthread_t descriptor;
#else
		void* descriptor;
#endif

	};
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	* ������
	*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	class DNET_EXPORT dnet_mutex_t
	{
	public:
		dnet_mutex_t();
		~dnet_mutex_t();
	public:
		void lock();
		void unlock();
	private:
		void* locker;
	};

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	* ��ȷ��ʱ����:��С��1ms��ʱ��ռ��CPU
	*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
	DNET_EXPORT void dnet_usleep(int useconds_);
	DNET_EXPORT void dnet_msleep(int mseconds_);
	DNET_EXPORT void dnet_psleep(int mseconds_, int useconds_);
	DNET_EXPORT void dnet_sleep();  //������CPUռ��Ȩ
									//�Ǿ�ȷ��ʱ,��ռ��CPU
	DNET_EXPORT void dnet_msleepx(int mseconds_);
	DNET_EXPORT void dnet_usleepx(int useconds_);
#undef DNET_EXPORT

#ifdef __cplusplus
}
#endif

#endif
