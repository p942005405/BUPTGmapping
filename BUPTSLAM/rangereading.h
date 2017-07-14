#ifndef RANGEREADING_H
#define RANGEREADING_H

#include <vector>
#include "sensorreading.h"
#include "rangesensor.h"

namespace GMapping{

/* �̳������ public std::vector<double>�� ��Ҫ�����ڱ��漤������  */
class RangeReading: public SensorReading, public std::vector<double>
{
	public:
		
		RangeReading(const RangeSensor* rs, double time=0);

		/* ���캯�� */
		RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time=0);
		
		virtual ~RangeReading();
		
		/* ��ȡ�����˵�λ����Ϣ */
		inline const OrientedPoint& getPose() const 
		{
			return m_pose;
		}

		/* ���û����˵�λ����Ϣ */
		inline void setPose(const OrientedPoint& pose) 
		{
			m_pose=pose;
		}
		
		/* ֱ�۵���⣺����һЩ����ܽ��ĵ�, �ڵ��õĹ����У����ֵ��������Ϊ�ֱ��� */
		unsigned int rawView(double* v, double density=0.) const;
		
		std::vector<Point> cartesianForm(double maxRange=1e6) const;
		
		unsigned int activeBeams(double density=0.) const;
	
protected:
		OrientedPoint m_pose;
};

};

#endif
