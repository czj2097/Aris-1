﻿#include "Platform.h"
#include "Aris_Message.h"

#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <algorithm>
#include <map>

#ifdef PLATFORM_IS_WINDOWS 
#include <Windows.h>
#endif
#ifdef PLATFORM_IS_LINUX 
#include <semaphore.h>
#endif

using namespace std;

namespace Aris
{
	namespace Core
	{
		class SEM
		{
		private:
			std::mutex _mutex;
			std::condition_variable _cv;
			volatile int _sig_value;

		public:
			SEM(const SEM & other) = delete;
			SEM &operator=(const SEM& other) = delete;
			SEM(SEM && other) = delete;
			SEM &operator=(SEM&& other) = delete;

			SEM(int iniValue = 0);
			~SEM();
			void Post();
			void Wait();

		};

		SEM::SEM(int iniValue)
		{
			std::lock_guard<std::mutex> lck(_mutex);
			_sig_value = iniValue;
		}
		SEM::~SEM()
		{
		}
		void SEM::Post()
		{
			std::lock_guard<std::mutex> lck(_mutex);
			_sig_value++;
			if (_sig_value==1)
			{
				_cv.notify_one();
			}
		}
		void SEM::Wait()
		{
			std::unique_lock<std::mutex> lck(_mutex);
			if (_sig_value == 0)
			{
				_cv.wait(lck);
			}
			_sig_value--;
		}
		
		std::queue<Aris::Core::MSG> msgq;
		
		SEM MsgReceived;
		std::mutex MsgMutex,MsgCallbackMutex;
		bool ifSkipLoop = false;
		bool isRunning = false;

		map < int, vector<function<int(Aris::Core::MSG &)> > > Msg_CallBack_Map;
		vector<function<int(Aris::Core::MSG &)> > DefaultCallBacks;

		void PostMsg(const Aris::Core::MSG &InMsg)
		{
			std::lock_guard<std::mutex> lck(MsgMutex);
			
			msgq.push(InMsg);
			MsgReceived.Post();
		}
		void GetMsg(Aris::Core::MSG &rMsg)
		{
			bool Flag=true;
			while(Flag)
			{
				MsgReceived.Wait();

				std::lock_guard<std::mutex> lck(MsgMutex);

				if(!msgq.empty())
				{
					rMsg.Swap(msgq.front());
					msgq.pop();
					Flag=false;
				}
			}
		}
		void RegisterMsgCallback(int message, function<int(Aris::Core::MSG &)> CallBack)
		{
			MsgCallbackMutex.lock();
			
			auto found = Msg_CallBack_Map.find(message);
			if (found == Msg_CallBack_Map.end())
			{
				vector<function<int(Aris::Core::MSG &)> > Msg_CallBacks;
				if (CallBack != nullptr)
				{
					Msg_CallBacks.push_back(CallBack);
				}
				Msg_CallBack_Map[message] = Msg_CallBacks;
			}
			else
			{
				found->second.clear();
				if (CallBack != nullptr)
				{
					found->second.push_back(CallBack);
				}
			}

			MsgCallbackMutex.unlock();
		}
		void RegisterDefaultCallback(std::function<int(Aris::Core::MSG &)> CallBack)
		{
			MsgCallbackMutex.lock();
			
			DefaultCallBacks.clear();
			if (CallBack != nullptr)
			{
				DefaultCallBacks.push_back(CallBack);
			}

			MsgCallbackMutex.unlock();
		}

		void RunMsgLoop()
		{
			if (isRunning)
				throw(std::logic_error("the msg loop is already started"));
			else
				isRunning = true;

			Aris::Core::MSG Msg;
			ifSkipLoop = false;

			while (GetMsg(Msg),ifSkipLoop==false)
			{
				std::lock_guard<std::mutex> lck(MsgCallbackMutex);

				auto found = Msg_CallBack_Map.find(Msg.GetMsgID());

				if (found == Msg_CallBack_Map.end())
				{
					for (auto func : DefaultCallBacks)
					{
						if (func != nullptr)
						{
							func(Msg);
						}
					}
				}
				else
				{
					for (auto func : found->second)
					{
						if (func != nullptr)
						{
							func(Msg);
						}
					}
				}
				
			}

			isRunning = false;
		}
		void StopMsgLoop()
		{
			if (isRunning)
			{
				ifSkipLoop = true;
				PostMsg(Core::MSG(0, 0));
			}
			else
			{
				throw(std::logic_error("the msg loop is not started, thus can not be stoped"));
			}
		}
		void ClearMsgLoop()
		{
			/*清空消息队列*/
			MsgCallbackMutex.lock();
			while (!msgq.empty())
			{
				msgq.pop();
			}
			MsgCallbackMutex.unlock();
		}
	}
}