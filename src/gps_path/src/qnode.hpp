/**
 * @file /include/tmp/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tmp_QNODE_HPP_
#define tmp_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "tableeditor.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/



/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init(TableEditor *ptr_table);
	void run();

private:
	ros::Subscriber m_gpsfix_sub;
  	int init_argc;
	char** init_argv;
};



#endif /* tmp_QNODE_HPP_ */
