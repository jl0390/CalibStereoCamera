#ifndef PQTHREAD_H
#define PQTHREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <chrono>

class PQThreadCallback
{
public:
	virtual void sendMessage(int msg, long param = 0, void *data = NULL) = 0;
};

class PQThread : public QThread
{
	Q_OBJECT

public:
	PQThread(QObject *parent = 0) : QThread(parent)
	{
		m_quit = false;
		m_pause = false;
		m_tid = 0;
	}
	virtual ~PQThread() {}

	static void destroy(PQThread **thread)
	{
		PQThread *t = *thread;
		if (t) {
			t->stop();
			delete t;
			*thread = NULL;
		}
	}

	void pause()
	{
		m_mutex.lock();
		m_pause = true;
		m_mutex.unlock();
		wake();
	}

	void resume()
	{
		m_mutex.lock();
		m_pause = false;
		m_mutex.unlock();
		wake();
	}

	void stop()
	{
		m_mutex.lock();
		m_pause = false;
		m_quit = true;
		m_mutex.unlock();
		wake();
		wait();
	}

	void waitCond(unsigned long time = ULONG_MAX)
	{
		m_mutex.lock();
		m_cond.wait(&m_mutex, time);
		m_mutex.unlock();
	}

	void wake()
	{
		m_cond.wakeAll();
	}

	bool quited()
	{
		QMutexLocker locker(&m_mutex);
		return m_quit;
	}

	bool paused()
	{
		QMutexLocker locker(&m_mutex);
		return m_pause;
	}

	void setCallback(PQThreadCallback *cb, int regist=1)
	{
		QMutexLocker locker(&m_mutex);
		if (regist) {
			m_callbacks.push_back(cb);
		}
		else {
			for (int i = 0; i < (int)m_callbacks.size(); ++i) {
				if (m_callbacks[i] == cb) {
					m_callbacks.erase(m_callbacks.begin() + i);
					break;
				}
			}
		}
	}

	void sendMessage(int msg, long param, void *data)
	{
		QMutexLocker locker(&m_mutex);
		for (int i = 0; i < (int)m_callbacks.size(); ++i) {
			if (m_callbacks[i])
				m_callbacks[i]->sendMessage(msg, param, data);
		}
	}

protected:
    virtual void run() = 0;

	int m_quit;
	int m_pause;
	QMutex m_mutex;
	QWaitCondition m_cond;
	QString m_tag;
	int m_tid;	//thread id

	std::vector<PQThreadCallback*> m_callbacks;
};

class PQTimer {
	using Clock = std::chrono::high_resolution_clock;
public:
	/*! \brief start or restart timer */
	inline void tic() {
		m_start = Clock::now();
	}
	/*! \brief stop timer */
	inline void toc() {
		m_end = Clock::now();
	}
	/*! \brief return time in ms */
	inline double elapsed() {
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(m_end - m_start);
		return duration.count();
	}

private:
	Clock::time_point m_start, m_end;
};

#endif // PQTHREAD_H
