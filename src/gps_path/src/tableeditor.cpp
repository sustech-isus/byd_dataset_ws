/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtWidgets>
#include "tableeditor.hpp"
#include <functional>
#include <fstream> 
//linux get HOME path
#include <stdlib.h>
//! [0]
#include "conversions.h"
#include <math.h> 
#include <string>     // std::string
#include <stdlib.h>
#include <limits.h>

std::vector<std::string> split(const std::string& s, const std::string& c);

TableEditor::TableEditor(const QString &tableName, QWidget *parent)
    : QWidget(parent)
{

    m_index=0;
    m_is_record=false;
    m_is_pause=false;
    m_current_road_type=0;
    m_current_save_type=0;

    std::string homepath = getenv("HOME");
    m_save_tmp_path=homepath+"/.haylion_gps.txt";


    m_tableWidget=new QTableWidget(this);
    m_tableWidget->setColumnCount(10);
    setupModel();
//! [1]

//! [2]
    recordButton = new QPushButton(tr("record"));
    pauseButton =new QPushButton(tr("pause"));
    //submitButton->setDefault(true);
    saveButton = new QPushButton(tr("save"));
    quitButton = new QPushButton(tr("Quit"));
    m_road_property=new QComboBox();  
    setupRoadProperty();
    m_road_label=new QLabel(" Road Type:");
    m_null_label=new QLabel(" ");
    m_save_label=new QLabel(" Save Type:");
    m_filter_step_label=new QLabel(" Step:");
    m_filter_combo=new QComboBox();
    m_filter_step_spinbox=new QDoubleSpinBox();
    setupFilters(); 
    //buttonBox = new QDialogButtonBox(Qt::Vertical);
    //buttonBox->addButton(m_road_property,QDialogButtonBox::ActionRole);
    //buttonBox->addButton(submitButton, QDialogButtonBox::ActionRole);
    //buttonBox->addButton(revertButton, QDialogButtonBox::ActionRole);
    //buttonBox->addButton(quitButton, QDialogButtonBox::RejectRole);
//! [2]

//! [3]
    connect(m_road_property,SIGNAL(currentIndexChanged(int)),this,SLOT(roadTypeChange(int)));
    connect(m_filter_combo,SIGNAL(currentIndexChanged(int)),this,SLOT(saveTypeChange(int)));
    connect(recordButton, SIGNAL(clicked()), this, SLOT(record()));
    connect(pauseButton, SIGNAL(clicked()), this, SLOT(pause()));
    connect(saveButton, SIGNAL(clicked()), this, SLOT(saveRecord()));
    connect(quitButton, SIGNAL(clicked()), this, SLOT(close()));
//! [3]
    QVBoxLayout *rightSideLayout=new QVBoxLayout;
    rightSideLayout->addWidget(recordButton);
    rightSideLayout->addWidget(pauseButton);
    rightSideLayout->addWidget(m_null_label);
    rightSideLayout->addWidget(m_save_label);
    rightSideLayout->addWidget(m_filter_combo);

    QHBoxLayout *filter_spin_Layout=new QHBoxLayout;
    filter_spin_Layout->addWidget(m_filter_step_label);
    filter_spin_Layout->addWidget(m_filter_step_spinbox);
    rightSideLayout->addLayout(filter_spin_Layout);
    rightSideLayout->addWidget(saveButton);

    rightSideLayout->addStretch();
    rightSideLayout->addWidget(m_road_label);
    rightSideLayout->addWidget(m_road_property);
    rightSideLayout->addStretch();
    rightSideLayout->addWidget(quitButton);
//! [4]
    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addWidget(m_tableWidget);
    mainLayout->addLayout(rightSideLayout);
    setLayout(mainLayout);
    setWindowTitle(tr("ZhiNengChiAI GPS Path"));

    resize(800,350);

}
//! [4]

//! [5]
void TableEditor::record()
{
    m_is_record=!m_is_record;
    if(m_is_record)
    {
        recordButton->setText("stop");
        startRecord();
    }
    else
    {
        recordButton->setText("record");
        pause();
        stopRecord();
    }

}

void TableEditor::pause()
{
    if(m_is_record)
    {
        m_is_pause=!m_is_pause;
        if(m_is_pause)
        {
             pauseButton->setText("resume");
        }
        else
        {
             pauseButton->setText("pause");
        }
    }
    else
    { 
        m_is_pause=false;
        pauseButton->setText("pause");
    }
    

}

void TableEditor::startRecord()
{
    m_save_tmp_stream.open(m_save_tmp_path.c_str());
}
void TableEditor::stopRecord()
{
    m_save_tmp_stream.close();
}
void TableEditor::roadTypeChange(int index)
{
    m_current_road_type=index+1;
}

void TableEditor::saveTypeChange(int index)
{
    m_current_save_type=index;
    //std::cout<<"filter type:"<<index<<std::endl;
}

void TableEditor::saveRecord()
{
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Save GPS Path"), "",
        tr("*.txt;;All Files (*)"));

    std::string std_filename=fileName.toStdString();
    if(fileName.isEmpty())
    {
        return;
    }
    else
    {
        if(m_current_save_type==0)
        {
            saveRaw(std_filename);
        }
        else
        {
            saveFiltered(std_filename);
        }
    }

}

void TableEditor::saveRaw(std::string file_path)
{
    std::ifstream srce(m_save_tmp_path.c_str(), std::ios::binary);
    std::ofstream dest(file_path.c_str(), std::ios::binary);
    dest << srce.rdbuf();

}

//void TableEditor::saveRaw(std::string file_path)
//{
//    std::ifstream srce(m_save_tmp_path.c_str());
//    std::ofstream dest(file_path.c_str());
//    std::string line_buf;
//
//    double last_northing=0;
//    double last_easting=0;
//    while(getline(srce,line_buf))
//    {
//        //std::cout<<line_buf;
//        std::vector<std::string> tokens = split(line_buf, ","); //可按多个字符来分隔;
//        if(tokens.size()<2)
//        {
//            continue;
//        }
//        double longitude=std::atof(tokens[0].c_str());
//        double latitude=std::atof(tokens[1].c_str());
//
//        double northing, easting;
//        std::string zone;
//
//        gps_common::LLtoUTM(latitude, longitude, northing, easting, zone);
//        double dn=northing-last_northing;
//        double de=easting-last_easting;
//        double dist=sqrt(dn*dn+de*de);
//       last_northing=northing;
//        last_easting=easting;
//        dest<<line_buf<<","<<dist<<std::endl;
//    }
//}

void TableEditor::saveFiltered(std::string file_path)
{
    std::ifstream srce(m_save_tmp_path.c_str());
    std::ofstream dest(file_path.c_str());
    std::string line_buf;
 
    std::vector<std::vector<double> > clusters;
    
    double filter_step=m_filter_step_spinbox->value();
    if(filter_step<=0)
    {
        filter_step=0.2;
    }
    //std::cout<<"fitler_step="<<filter_step<<std::endl;

    unsigned int max_data_lenght=0;

    double last_northing=0;
    double last_easting=0;
    while(getline(srce,line_buf))
    {
        //std::cout<<line_buf;
        std::vector<std::string> tokens = split(line_buf, ","); //可按多个字符来分隔;
        if(tokens.size()<2)
        {
            continue;
        }
        double longitude=std::atof(tokens[0].c_str());
        double latitude=std::atof(tokens[1].c_str());

        double northing, easting;
        std::string zone;

        gps_common::LLtoUTM(latitude, longitude, northing, easting, zone);
        double dn=northing-last_northing;
        double de=easting-last_easting;
        double dist=sqrt(dn*dn+de*de);


        if(dist<filter_step)
        {
            std::vector<double> msg_data;
            msg_data.push_back(longitude);
            msg_data.push_back(latitude);
            for(int i=2;i<tokens.size();i++) 
            {
                msg_data.push_back(std::atof(tokens[i].c_str()));
            }
            //std::cout<<"msg_data.size()="<<msg_data.size()<<std::endl;
            if(msg_data.size()>max_data_lenght)
            {
                max_data_lenght=msg_data.size();
            }
            clusters.push_back(msg_data);
        }
        else
        {
            last_northing=northing;
            last_easting=easting;
            if(clusters.size()>0)
            {
                 //std::cout<<"cluster.size()="<<clusters.size()<<",max_data_lenght="<<max_data_lenght<<std::endl;
                 for(int i=0;i<max_data_lenght;i++)
                 {
                      double tmp_val=0;
                      unsigned cnt_skip=0;
                      for(int c=0;c<clusters.size();c++)
                      {
                           if(clusters[c].size()<max_data_lenght)
                           {
                            
                               cnt_skip++;
                               continue;
                           }

                           //if(i==3)
                           //{
                            //  std::cout<<clusters[c][i]<<",";
                           //}

                           tmp_val=tmp_val+clusters[c][i];
                      }
                      double used_cnt=clusters.size()-cnt_skip;
                      tmp_val=tmp_val/used_cnt;

                      //if(i==3)
                      //{
                      //    std::cout<<"result="<<tmp_val<<std::endl;
                      //}

                      if(i<2)
                      {
                          dest<<std::setprecision(10)<<tmp_val<<",";
                      }
                      else if(i==(max_data_lenght-1))
                      {
                          dest<<tmp_val<<std::endl;
                      }
                      else
                      {
                          dest<<tmp_val<<",";
                      }

                 }
                 clusters.clear();
                 max_data_lenght=0;
            }
             
            dest<<line_buf<<std::endl;
        }
        //std::cout<<dist<<std::endl;
 
    }
}

void TableEditor::setupRoadProperty()
{

    m_road_property->addItem("StraightHigh", 0);
    m_road_property->addItem("StraightMid", 1);
    m_road_property->addItem("StraightLow", 2);
    m_road_property->addItem("TurnLeft", 3);
    m_road_property->addItem("TurnRight", 4);
    m_road_property->addItem("ChangeRoad", 5);
    m_current_road_type=1;
}

void TableEditor::setupFilters()
{
    m_filter_combo->addItem("raw", 0);
    m_filter_combo->addItem("filtered", 1);

    m_filter_step_spinbox->setRange(0, 100);  // 范围
    m_filter_step_spinbox->setDecimals(2);  // 精度
    m_filter_step_spinbox->setSingleStep(0.01); // 步长

    m_filter_step_spinbox->setValue(0.2);
}

//! [5]
//!
void TableEditor::setupModel()
{
    //! [9]
    // Set up row and column names
    QStringList items;
    items <<"Index"<<"Longitude" << "Latitude" << "Heading"<< "Speed" <<  "Satellites" << "Status" <<"Road Type";
    QStringList messages;
    messages << "Message 0" << "Message 1" << "Message 2" << "Message 3" << "Message 4" << "Message 5"<< "Message 6" << "Message 7" << "Message 8" << "Message 9";

    m_table_row=10;
    m_table_col=8;
    //! [11]
    m_tableWidget->setRowCount(m_table_row);
    m_tableWidget->setColumnCount(m_table_col);
    m_tableWidget->setHorizontalHeaderLabels(items);
    m_tableWidget->verticalHeader()->hide();
    //m_tableWidget->setVerticalHeaderLabels(messages);
    m_tableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_tableWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_tableWidget->setCurrentCell(-1, -1);
    m_tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tableWidget->resizeColumnsToContents();
    m_tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

}


void TableEditor::callback(const gps_common::GPSFix& fix)
{
    std::vector<double> row_data;
    row_data.push_back(m_index);
    row_data.push_back(fix.longitude);
    row_data.push_back(fix.latitude);
    row_data.push_back(fix.track);
    row_data.push_back(fix.speed);
    row_data.push_back(fix.status.satellites_visible);
    row_data.push_back(fix.status.status);
    row_data.push_back(m_current_road_type);
    m_index++;

    if((m_is_record)&&(!m_is_pause))
    {
        m_save_tmp_stream<<std::setprecision(10)<<fix.longitude<<","<<std::setprecision(10)<<fix.latitude<<","<<m_current_road_type<<","<<fix.track<<","<<fix.speed<<","<<"0\n";
    }
    //for(int i=0;i<row_data.size();i++)
    //{
    //    std::cout<<row_data[i]<<" ";
    //}
    //std::cout<<"\n";

    m_table_datas.push_back(row_data);
    if(m_table_datas.size()>m_table_row)
    {
         m_table_datas.pop_front();
    }

    for(int row=0;row<m_table_datas.size();row++)
    {
        std::vector<double> tmp_row_data=m_table_datas[row];
        for(int col=0;col<m_table_col;col++)
        {  
            QModelIndex index = m_tableWidget->model()->index(row, col);
            if((col==0)||(col>=5))
            {
                m_tableWidget->model()->setData(index,(int)(tmp_row_data[col]));
            }
            else
            {
                m_tableWidget->model()->setData(index,QString::number(tmp_row_data[col], 'f',10));
            }
        }
    }
    m_tableWidget->viewport()->update();
}


std::vector<std::string> split(const std::string& s, const std::string& c)
{
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(std::string::npos != pos2)
    {
       v.push_back(s.substr(pos1, pos2-pos1));
 
       pos1 = pos2 + c.size();
       pos2 = s.find(c, pos1);
    }
	
    if(pos1 != s.length())
    {
       v.push_back(s.substr(pos1));
    }
    return v;
}
