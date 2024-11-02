#* @Kelompok
#* || ACH FAQIH AINUL MUQORROBIN 221910201076
#* || RIZAL ADITYA WICAKSONO 221910201053
#* || MUHAMMAD SYARIFUDDIN JUHDI 221910201081
#* || Erico Dwi Novianto 221910201108


import customtkinter
import matplotlib.pyplot as plt
from coba import Kinematics
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

class App:
    def __init__(self):
        self.linkA = 0
        self.linkB = 0
        self.linkC = 0

        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.endX = 0
        self.endY = 0
        self.endT = 0

        customtkinter.set_appearance_mode("Light")

        app = customtkinter.CTk()
        app.title("Manipulator 3 DOF")
        app.configure(fg_color='#fff', font=('DM Sans', 13))

        #* Container Frame
        plotFrame = customtkinter.CTkFrame(app, fg_color='#fff', corner_radius=5)
        plotFrame.grid(row=0, column=0, padx=8, pady=(8,0))

        displayFrame = customtkinter.CTkFrame(app, fg_color='#e2e8f0', corner_radius=5)
        displayFrame.grid(row=1, column=0, padx=8, pady=(0,8))

        controllFrame = customtkinter.CTkFrame(app, fg_color='#fff', corner_radius=5)
        controllFrame.grid(row=2, column=0, padx=8, pady=(0,12))

        frameA = customtkinter.CTkFrame(controllFrame, fg_color='#e2e8f0', corner_radius=5)
        frameA.grid(row=0, column=0, padx=(8,4))

        frameB = customtkinter.CTkFrame(controllFrame, fg_color='#e2e8f0', corner_radius=5)
        frameB.grid(row=0, column=1, padx=(4,8))


        #* Matplotlib canvas
        self.fig, self.ax = plt.subplots(figsize=(5, 3))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plotFrame)
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.grid(row=0, column=0, padx=12, pady=12, columnspan=2)


        #* Data display
        labelLinkA = customtkinter.CTkLabel(displayFrame, text=f'Link A', width=46, anchor='w')
        labelLinkA.grid(row=0, column=0, padx=12, pady=(4,2))
        self.linkA = customtkinter.CTkEntry(displayFrame, width=46, corner_radius=5, fg_color='#fff', border_width=0)
        self.linkA.grid(row=0, column=1, padx=12, pady=(4,2))

        labelLinkB = customtkinter.CTkLabel(displayFrame, text=f'Link B', width=46, anchor='w')
        labelLinkB.grid(row=1, column=0, padx=12, pady=(4,2))
        self.linkB = customtkinter.CTkEntry(displayFrame, width=46, corner_radius=5, fg_color='#fff', border_width=0)
        self.linkB.grid(row=1, column=1, padx=12, pady=(4,2))
        
        labelLinkC = customtkinter.CTkLabel(displayFrame, text=f'Link C', width=46, anchor='w')
        labelLinkC.grid(row=2, column=0, padx=12, pady=(4,2))
        self.linkC = customtkinter.CTkEntry(displayFrame, width=46, corner_radius=5, fg_color='#fff', border_width=0)
        self.linkC.grid(row=2, column=1, padx=12, pady=(4,2))

        labelJoint1 = customtkinter.CTkLabel(displayFrame, text=f'Joint A', width=46, anchor='w')
        labelJoint1.grid(row=0, column=3, padx=12, pady=(4,2))
        self.joint1 = customtkinter.CTkLabel(displayFrame, text='0', width=46, anchor='w')
        self.joint1.grid(row=0, column=4, padx=12, pady=(4,2))

        labelJoint2 = customtkinter.CTkLabel(displayFrame, text=f'Joint B', width=46, anchor='w')
        labelJoint2.grid(row=1, column=3, padx=12, pady=(4,2))
        self.joint2 = customtkinter.CTkLabel(displayFrame, text='0', width=46, anchor='w')
        self.joint2.grid(row=1, column=4, padx=12, pady=(4,2))
        
        labelJoint3 = customtkinter.CTkLabel(displayFrame, text=f'Joint C', width=46, anchor='w')
        labelJoint3.grid(row=2, column=3, padx=12, pady=(4,2))
        self.joint3 = customtkinter.CTkLabel(displayFrame, text='0', width=46, anchor='w')
        self.joint3.grid(row=2, column=4, padx=12, pady=(4,2))

        labelEndX = customtkinter.CTkLabel(displayFrame, text=f'End X', width=46, anchor='w')
        labelEndX.grid(row=0, column=5, padx=12, pady=(4,2))
        self.endX = customtkinter.CTkLabel(displayFrame, text='0', width=46, anchor='w')
        self.endX.grid(row=0, column=6, padx=12, pady=(4,2))

        labelEndY = customtkinter.CTkLabel(displayFrame, text=f'End Y', width=46, anchor='w')
        labelEndY.grid(row=1, column=5, padx=12, pady=(4,2))
        self.endY = customtkinter.CTkLabel(displayFrame, text='0', width=46, anchor='w')
        self.endY.grid(row=1, column=6, padx=12, pady=(4,2))
        
        labelEndT = customtkinter.CTkLabel(displayFrame, text=f'End θ', width=46, anchor='w')
        labelEndT.grid(row=2, column=5, padx=12, pady=(4,2))
        self.endT = customtkinter.CTkLabel(displayFrame, text='0', width=46, anchor='w')
        self.endT.grid(row=2, column=6, padx=12, pady=(4,2))


        #* Forward kinematics frame
        labelJointA = customtkinter.CTkLabel(frameA, text='Joint A (Degrees)')
        labelJointA.grid(row=1, column=0, padx=12, pady=(4,2))
        self.thetaA = customtkinter.CTkEntry(frameA, width=185, corner_radius=5, fg_color='#fff', border_width=0)
        self.thetaA.grid(row=2, column=0, padx=12, pady=2)

        labelJointB = customtkinter.CTkLabel(frameA, text='Joint B (Degrees)')
        labelJointB.grid(row=3, column=0, padx=12, pady=2)
        self.thetaB = customtkinter.CTkEntry(frameA, width=185, corner_radius=5, fg_color='#fff', border_width=0)
        self.thetaB.grid(row=4, column=0, padx=12, pady=2)
        
        labelJointC = customtkinter.CTkLabel(frameA, text='Joint C (Degrees)')
        labelJointC.grid(row=5, column=0, padx=12, pady=2)
        self.thetaC = customtkinter.CTkEntry(frameA, width=185, corner_radius=5, fg_color='#fff', border_width=0)
        self.thetaC.grid(row=6, column=0, padx=12, pady=2)

        buttonA = customtkinter.CTkButton(frameA, text='Forward', command=self.forward, width=185, corner_radius=5, fg_color='#0f172a')
        buttonA.grid(row=7, column=0, padx=12, pady=(12,8))


        #* Inverse kinematics frame
        labelX = customtkinter.CTkLabel(frameB, text='X Coordinate')
        labelX.grid(row=1, column=1, padx=12, pady=(4,2))
        self.x = customtkinter.CTkEntry(frameB, width=185, corner_radius=5, fg_color='#fff', border_width=0)
        self.x.grid(row=2, column=1, padx=12, pady=2)

        labelY = customtkinter.CTkLabel(frameB, text='Y Coordinate')
        labelY.grid(row=3, column=1, padx=12, pady=(4,2))
        self.y = customtkinter.CTkEntry(frameB, width=185, corner_radius=5, fg_color='#fff', border_width=0)
        self.y.grid(row=4, column=1, padx=12, pady=2)
        
        labelT = customtkinter.CTkLabel(frameB, text= "Juhdi,Faqih,Erico,Rizal")
        labelT.grid(row=5, column=1, padx=12, pady=(4,2))
        labelT1 = customtkinter.CTkLabel(frameB, text= "1081,1076,1108,1053")
        labelT1.grid(row=6, column=1, padx=12, pady=(4,2))
        # self.t = customtkinter.CTkEntry(frameB, width=185, corner_radius=5, fg_color='#fff', border_width=0)
        # self.t.grid(row=6, column=1, padx=12, pady=2)

        buttonB = customtkinter.CTkButton(frameB, text='Inverse', command=self.inverse, width=185, corner_radius=5, fg_color='#0f172a')
        buttonB.grid(row=7, column=1, padx=12, pady=(12,8))

        app.mainloop() 

    def forward(self):
        jointAngle1 = float(self.thetaA.get())
        jointAngle2 = float(self.thetaB.get())
        jointAngle3 = float(self.thetaC.get())

        linkA = float(self.linkA.get())
        linkB = float(self.linkB.get())
        linkC = float(self.linkC.get())

        self.x.delete(0,'end')
        self.y.delete(0,'end')
        # self.t.delete(0,'end')

        self.robot = Kinematics(linkA, linkB, linkC)

        #* forward kinematics
        baseX, baseY, joint1X, joint1Y, joint2X, joint2Y, endEffectorX, endEffectorY = self.robot.forward(jointAngle1, jointAngle2, jointAngle3)

        self.joint1.configure(text=round(jointAngle1,2))
        self.joint2.configure(text=round(jointAngle2,2))
        self.joint3.configure(text=round(jointAngle3,2))
        self.endX.configure(text=round(endEffectorX,2))
        self.endY.configure(text=round(endEffectorY,2))
        self.endT.configure(text=round(jointAngle1+jointAngle2+jointAngle3,2))

        self.ax.clear()

        #* plot link 1
        self.ax.plot([baseX, joint1X], [baseY, joint1Y], 'b-', linewidth=3, label='Link 1')
        #* plot link 2
        self.ax.plot([joint1X, joint2X], [joint1Y, joint2Y], 'g-', linewidth=3, label='link 2')
        #* plot link 2
        self.ax.plot([joint2X, endEffectorX], [joint2Y, endEffectorY], 'r-', linewidth=3, label='link 2')
        #* plot link 3
        self.ax.plot(endEffectorX, endEffectorY, 'ro', markersize=10, label='End Effector')

        #* set limit grafik matplotlib
        if linkA >= linkB:
            limit = linkA * 2
        elif linkA < linkB:
            limit = linkB * 2

        self.ax.set_xlim(-(limit*2), limit*2)
        self.ax.set_ylim(-(limit*2), limit*2)

        #* label matplotlib
        self.ax.set_xlabel('X-axis')
        self.ax.set_ylabel('Y-axis')
        self.ax.set_title('2-DOF Robot Arm')

        self.ax.legend()
        self.canvas.draw()

    def inverse(self):
        x = float(self.x.get())
        y = float(self.y.get())
        t = float(self.y.get())

        linkA = float(self.linkA.get())
        linkB = float(self.linkB.get())
        linkC = float(self.linkC.get())

        self.thetaA.delete(0,'end')
        self.thetaB.delete(0,'end')
        self.thetaC.delete(0,'end')

        self.robot = Kinematics(linkA, linkB,linkC)
        
        jointAngleA, jointAngleB,jointAngleC = self.robot.inverse(x, y, t)
        baseX, baseY, joint1X, joint1Y, joint2X, joint2Y, endEffectorX, endEffectorY = self.robot.forward(jointAngleA, jointAngleB, jointAngleC)

        self.joint1.configure(text=round(jointAngleA,2))
        self.joint2.configure(text=round(jointAngleB,2))
        self.joint3.configure(text=round(jointAngleC,2))
        self.endX.configure(text=round(endEffectorX,2))
        self.endY.configure(text=round(endEffectorY,2))
        self.endT.configure(text=round(jointAngleA+jointAngleB+jointAngleB,2))

        self.ax.clear()

        #* plot link 1
        self.ax.plot([baseX, joint1X], [baseY, joint1Y], 'b-', linewidth=3, label='Link 1')
        #* plot link 2
        self.ax.plot([joint1X, joint2X], [joint1Y, joint2Y], 'g-', linewidth=3, label='link 2')
        #* plot link 2
        self.ax.plot([joint2X, endEffectorX], [joint2Y, endEffectorY], 'r-', linewidth=3, label='link 2')
        #* plot link 3
        self.ax.plot(endEffectorX, endEffectorY, 'ro', markersize=10, label='End Effector')

        #* set limit grafik matplotlib
        if linkA >= linkB:
            limit = linkA * 2
        elif linkA < linkB:
            limit = linkB * 2

        self.ax.set_xlim(-(limit*2), limit*2)
        self.ax.set_ylim(-(limit*2), limit*2)

        #* label matplotlib
        self.ax.set_xlabel('X-axis')
        self.ax.set_ylabel('Y-axis')
        self.ax.set_title('2-DOF Robot Arm')

        self.ax.legend()
        self.canvas.draw()


app = App()