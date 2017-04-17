#!/bin/sh

sed -i "s/<var>/<var units=\"s\">/g" DP_Product/DP_rt_frame.xml

sed -i "s/<var>/<var units=\"s\">/g" DP_Product/DP_rt_timeline.xml
sed -i "s/<var line_color/<var units=\"--\" line_color/g" DP_Product/DP_rt_timeline.xml
sed -i "s/trick_frame_log.frame_log.job_user_id/frame_log.frame_log.job_user_id/g" DP_Product/DP_rt_timeline.xml

sed -i "s/<var>/<var units=\"s\">/g" DP_Product/DP_rt_timeline_init.xml
sed -i "s/<var line_color/<var units=\"--\" line_color/g" DP_Product/DP_rt_timeline_init.xml
sed -i "s/trick_frame_log.frame_log.job_user_id/frame_log.frame_log.job_user_id/g" DP_Product/DP_rt_timeline_init.xml

sed -i "s/<var/<var units=\"s\"/g"   DP_Product/DP_rt_trickjobs.xml
sed -i "s/<var/<var units=\"s\"/g"   DP_Product/DP_rt_userjobs.xml
