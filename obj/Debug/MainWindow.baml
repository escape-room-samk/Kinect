   M S B A M L   `   `   `  ����  BodyBasics-WPF4    .Microsoft.Samples.Kinect.BodyBasics.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  Body Basics��5   	   $
��1080��5   	   $
��1920��6    � 	Resources 5         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush��%�   &	        &	 #     &	 F     �� 5	      $ 	#ff6e6e6e��6.   �� 5
      $ 	#ff52318f��60   �� 5      $ 	#ff00BCF2��6.   5      .��5      � -   5        $
��grid��5   	   $��	10 0 10 0q�6    � RowDefinitions 5   
   �� 5      $
��Auto �6   �� 5      $��* �6   �� 5      $��* �6   �� 5      $
��Auto �6    � ColumnDefinitions 5   
   �� 5      $��3* �6   �� 5      $��2* �6   .+�5      �� $��0��6   $��2��6   $��Images\Logo.png��6-   $
��Left=�6F   $
��Fill=�6a   $��32��6p   $��81��6|   $��0 10 0 5q�6�   �� 5      $��0��6   $��0 0 -1 0q�6   $��Right=�6/   $��Bottom=�6K   #��[  6f   $��Segoe UI)�6�   $��18(�6�   Body Basics6�   �� 5      $��0��6   $��Images\Status.png��6   $
��None=�64   $��Center=�6C   $��0 0 0 5q�6`   (� 5      $��1��6   $��2��6   $��Center=�6,   .��5      �� $��UniformToFill=�6-   ��6   �� *ImageSource+6-   5       �� 5"   
   $��1��6   $��1��6    $��Center=�60   $��Stretch=�6K   .@�5#      �� -   5        $��	lblLetter��5#      .��6%   " Letter you are showing right now6G   �� 5$      -   5        $��	tbxLetter��5$      $��34(�6'   $��Stretch=�65   $��Center=�6S   6p   5%      �� 5'   
   $��2��6   $��1��6    $��Center=�60   $��Stretch=�6K   .@�5(      �� -   5        $��lblWord��5(      .��6#   )'Full word that will be send to database6L   �� 5)      -   5        $��tbxrighthand��5)      $��34(�6*   $��Stretch=�68   $��Center=�6V   6s   �� 5*      -   5        $��lblDataSend��5*      $��30(�6'   67   5+      �� 5.   
   -   5        $��	statusBar��5.   S   $��3��6   $��2��6!   $��Stretch=�65   $��Bottom=�6d   
�������#��[  6�   .@�5/      �� ��6   �� *
StatusText+50      52      53      