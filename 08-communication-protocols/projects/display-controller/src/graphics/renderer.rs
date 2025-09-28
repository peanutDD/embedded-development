//! 图形渲染器模块
//!
//! 提供高效的2D图形渲染功能，支持多种几何图形绘制、图像处理和特效。
//! 包含基本图形绘制、复杂路径渲染、图像合成和性能优化功能。

use super::*;
use embedded_graphics::{
    pixelcolor::{BinaryColor, Rgb565, Rgb888},
    prelude::*,
    primitives::{
        Circle, Ellipse, Line, Polygon, Polyline, Rectangle, RoundedRectangle, Triangle,
        PrimitiveStyle, PrimitiveStyleBuilder,
    },
    text::{Baseline, Text, TextStyleBuilder},
    image::{Image, ImageRaw},
    draw_target::DrawTarget,
    geometry::{Dimensions, OriginDimensions},
    Drawable,
};
use heapless::{Vec, String};
use micromath::F32Ext;

/// 渲染目标trait
pub trait RenderTarget {
    /// 获取尺寸
    fn size(&self) -> Size;
    
    /// 设置像素
    fn set_pixel(&mut self, x: u16, y: u16, color: Color) -> Result<(), GraphicsError>;
    
    /// 获取像素
    fn get_pixel(&self, x: u16, y: u16) -> Result<Color, GraphicsError>;
    
    /// 清除屏幕
    fn clear(&mut self, color: Color) -> Result<(), GraphicsError>;
    
    /// 刷新显示
    fn flush(&mut self) -> Result<(), GraphicsError>;
    
    /// 设置裁剪区域
    fn set_clip_rect(&mut self, rect: Option<Rectangle>);
    
    /// 获取裁剪区域
    fn get_clip_rect(&self) -> Option<Rectangle>;
}

/// 内存渲染目标
pub struct MemoryRenderTarget {
    /// 图像数据
    image_data: ImageData,
    /// 裁剪区域
    clip_rect: Option<Rectangle>,
}

impl MemoryRenderTarget {
    /// 创建新的内存渲染目标
    pub fn new(width: u16, height: u16, format: ColorFormat) -> Result<Self, GraphicsError> {
        let info = ImageInfo::new(width, height, format);
        let image_data = ImageData::new(info)?;
        
        Ok(Self {
            image_data,
            clip_rect: None,
        })
    }
    
    /// 获取图像数据
    pub fn image_data(&self) -> &ImageData {
        &self.image_data
    }
    
    /// 获取可变图像数据
    pub fn image_data_mut(&mut self) -> &mut ImageData {
        &mut self.image_data
    }
    
    /// 检查点是否在裁剪区域内
    fn is_point_clipped(&self, x: u16, y: u16) -> bool {
        if let Some(clip) = self.clip_rect {
            let point = Point::new(x as i32, y as i32);
            !graphics_utils::point_in_rect(point, clip)
        } else {
            false
        }
    }
}

impl RenderTarget for MemoryRenderTarget {
    fn size(&self) -> Size {
        Size::new(self.image_data.info.width as u32, self.image_data.info.height as u32)
    }
    
    fn set_pixel(&mut self, x: u16, y: u16, color: Color) -> Result<(), GraphicsError> {
        if self.is_point_clipped(x, y) {
            return Ok(());
        }
        self.image_data.set_pixel(x, y, color)
    }
    
    fn get_pixel(&self, x: u16, y: u16) -> Result<Color, GraphicsError> {
        self.image_data.get_pixel(x, y)
    }
    
    fn clear(&mut self, color: Color) -> Result<(), GraphicsError> {
        self.image_data.clear(color);
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), GraphicsError> {
        // 内存目标不需要刷新
        Ok(())
    }
    
    fn set_clip_rect(&mut self, rect: Option<Rectangle>) {
        self.clip_rect = rect;
    }
    
    fn get_clip_rect(&self) -> Option<Rectangle> {
        self.clip_rect
    }
}

/// 图形渲染器
pub struct GraphicsRenderer {
    /// 当前变换矩阵
    transform: Transform2D,
    /// 变换矩阵栈
    transform_stack: Vec<Transform2D, 16>,
    /// 当前绘制参数
    draw_params: DrawParams,
    /// 当前文本参数
    text_params: TextParams,
}

/// 2D变换矩阵
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform2D {
    /// 矩阵元素 [a, b, c, d, tx, ty]
    /// | a  c  tx |
    /// | b  d  ty |
    /// | 0  0  1  |
    pub matrix: [f32; 6],
}

impl Default for Transform2D {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform2D {
    /// 单位矩阵
    pub fn identity() -> Self {
        Self {
            matrix: [1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        }
    }
    
    /// 平移变换
    pub fn translate(tx: f32, ty: f32) -> Self {
        Self {
            matrix: [1.0, 0.0, 0.0, 1.0, tx, ty],
        }
    }
    
    /// 缩放变换
    pub fn scale(sx: f32, sy: f32) -> Self {
        Self {
            matrix: [sx, 0.0, 0.0, sy, 0.0, 0.0],
        }
    }
    
    /// 旋转变换（弧度）
    pub fn rotate(angle: f32) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self {
            matrix: [cos_a, sin_a, -sin_a, cos_a, 0.0, 0.0],
        }
    }
    
    /// 矩阵乘法
    pub fn multiply(&self, other: &Transform2D) -> Transform2D {
        let a1 = self.matrix[0];
        let b1 = self.matrix[1];
        let c1 = self.matrix[2];
        let d1 = self.matrix[3];
        let tx1 = self.matrix[4];
        let ty1 = self.matrix[5];
        
        let a2 = other.matrix[0];
        let b2 = other.matrix[1];
        let c2 = other.matrix[2];
        let d2 = other.matrix[3];
        let tx2 = other.matrix[4];
        let ty2 = other.matrix[5];
        
        Transform2D {
            matrix: [
                a1 * a2 + c1 * b2,
                b1 * a2 + d1 * b2,
                a1 * c2 + c1 * d2,
                b1 * c2 + d1 * d2,
                a1 * tx2 + c1 * ty2 + tx1,
                b1 * tx2 + d1 * ty2 + ty1,
            ],
        }
    }
    
    /// 变换点
    pub fn transform_point(&self, point: Point) -> Point {
        let x = point.x as f32;
        let y = point.y as f32;
        
        let new_x = self.matrix[0] * x + self.matrix[2] * y + self.matrix[4];
        let new_y = self.matrix[1] * x + self.matrix[3] * y + self.matrix[5];
        
        Point::new(new_x as i32, new_y as i32)
    }
    
    /// 变换尺寸
    pub fn transform_size(&self, size: Size) -> Size {
        let w = size.width as f32;
        let h = size.height as f32;
        
        // 计算变换后的四个角点
        let corners = [
            self.transform_point(Point::new(0, 0)),
            self.transform_point(Point::new(w as i32, 0)),
            self.transform_point(Point::new(0, h as i32)),
            self.transform_point(Point::new(w as i32, h as i32)),
        ];
        
        // 计算边界框
        let min_x = corners.iter().map(|p| p.x).min().unwrap_or(0);
        let max_x = corners.iter().map(|p| p.x).max().unwrap_or(0);
        let min_y = corners.iter().map(|p| p.y).min().unwrap_or(0);
        let max_y = corners.iter().map(|p| p.y).max().unwrap_or(0);
        
        Size::new((max_x - min_x) as u32, (max_y - min_y) as u32)
    }
    
    /// 获取逆矩阵
    pub fn inverse(&self) -> Option<Transform2D> {
        let a = self.matrix[0];
        let b = self.matrix[1];
        let c = self.matrix[2];
        let d = self.matrix[3];
        let tx = self.matrix[4];
        let ty = self.matrix[5];
        
        let det = a * d - b * c;
        if det.abs() < 1e-10 {
            return None; // 矩阵不可逆
        }
        
        let inv_det = 1.0 / det;
        
        Some(Transform2D {
            matrix: [
                d * inv_det,
                -b * inv_det,
                -c * inv_det,
                a * inv_det,
                (c * ty - d * tx) * inv_det,
                (b * tx - a * ty) * inv_det,
            ],
        })
    }
}

impl GraphicsRenderer {
    /// 创建新的渲染器
    pub fn new() -> Self {
        Self {
            transform: Transform2D::identity(),
            transform_stack: Vec::new(),
            draw_params: DrawParams::default(),
            text_params: TextParams::default(),
        }
    }
    
    /// 设置绘制参数
    pub fn set_draw_params(&mut self, params: DrawParams) {
        self.draw_params = params;
    }
    
    /// 获取绘制参数
    pub fn draw_params(&self) -> &DrawParams {
        &self.draw_params
    }
    
    /// 设置文本参数
    pub fn set_text_params(&mut self, params: TextParams) {
        self.text_params = params;
    }
    
    /// 获取文本参数
    pub fn text_params(&self) -> &TextParams {
        &self.text_params
    }
    
    /// 保存当前变换
    pub fn save_transform(&mut self) -> Result<(), GraphicsError> {
        self.transform_stack.push(self.transform)
            .map_err(|_| GraphicsError::OutOfMemory)
    }
    
    /// 恢复变换
    pub fn restore_transform(&mut self) -> Result<(), GraphicsError> {
        if let Some(transform) = self.transform_stack.pop() {
            self.transform = transform;
            Ok(())
        } else {
            Err(GraphicsError::InvalidParameter)
        }
    }
    
    /// 重置变换
    pub fn reset_transform(&mut self) {
        self.transform = Transform2D::identity();
    }
    
    /// 平移
    pub fn translate(&mut self, tx: f32, ty: f32) {
        let translate_matrix = Transform2D::translate(tx, ty);
        self.transform = self.transform.multiply(&translate_matrix);
    }
    
    /// 缩放
    pub fn scale(&mut self, sx: f32, sy: f32) {
        let scale_matrix = Transform2D::scale(sx, sy);
        self.transform = self.transform.multiply(&scale_matrix);
    }
    
    /// 旋转（弧度）
    pub fn rotate(&mut self, angle: f32) {
        let rotate_matrix = Transform2D::rotate(angle);
        self.transform = self.transform.multiply(&rotate_matrix);
    }
    
    /// 绘制像素
    pub fn draw_pixel<T: RenderTarget>(
        &self,
        target: &mut T,
        x: i32,
        y: i32,
        color: Color,
    ) -> Result<(), GraphicsError> {
        let point = self.transform.transform_point(Point::new(x, y));
        let size = target.size();
        
        if point.x >= 0 && point.x < size.width as i32 && point.y >= 0 && point.y < size.height as i32 {
            target.set_pixel(point.x as u16, point.y as u16, color)
        } else {
            Ok(())
        }
    }
    
    /// 绘制线段
    pub fn draw_line<T: RenderTarget>(
        &self,
        target: &mut T,
        start: Point,
        end: Point,
    ) -> Result<(), GraphicsError> {
        let color = self.draw_params.stroke_color.unwrap_or(Color::BLACK);
        let width = self.draw_params.stroke_width;
        
        // 变换端点
        let start_transformed = self.transform.transform_point(start);
        let end_transformed = self.transform.transform_point(end);
        
        // Bresenham直线算法
        self.draw_line_bresenham(target, start_transformed, end_transformed, color, width)
    }
    
    /// Bresenham直线算法
    fn draw_line_bresenham<T: RenderTarget>(
        &self,
        target: &mut T,
        start: Point,
        end: Point,
        color: Color,
        width: u8,
    ) -> Result<(), GraphicsError> {
        let mut x0 = start.x;
        let mut y0 = start.y;
        let x1 = end.x;
        let y1 = end.y;
        
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;
        
        loop {
            // 绘制粗线
            if width > 1 {
                let half_width = width as i32 / 2;
                for dy in -half_width..=half_width {
                    for dx in -half_width..=half_width {
                        let px = x0 + dx;
                        let py = y0 + dy;
                        let size = target.size();
                        if px >= 0 && px < size.width as i32 && py >= 0 && py < size.height as i32 {
                            target.set_pixel(px as u16, py as u16, color)?;
                        }
                    }
                }
            } else {
                let size = target.size();
                if x0 >= 0 && x0 < size.width as i32 && y0 >= 0 && y0 < size.height as i32 {
                    target.set_pixel(x0 as u16, y0 as u16, color)?;
                }
            }
            
            if x0 == x1 && y0 == y1 {
                break;
            }
            
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x0 += sx;
            }
            if e2 < dx {
                err += dx;
                y0 += sy;
            }
        }
        
        Ok(())
    }
    
    /// 绘制矩形
    pub fn draw_rectangle<T: RenderTarget>(
        &self,
        target: &mut T,
        rect: Rectangle,
    ) -> Result<(), GraphicsError> {
        match self.draw_params.draw_style {
            DrawStyle::Fill => self.fill_rectangle(target, rect),
            DrawStyle::Stroke => self.stroke_rectangle(target, rect),
            DrawStyle::FillStroke => {
                self.fill_rectangle(target, rect)?;
                self.stroke_rectangle(target, rect)
            }
        }
    }
    
    /// 填充矩形
    fn fill_rectangle<T: RenderTarget>(
        &self,
        target: &mut T,
        rect: Rectangle,
    ) -> Result<(), GraphicsError> {
        let color = self.draw_params.fill_color.unwrap_or(Color::WHITE);
        
        for y in 0..rect.size.height {
            for x in 0..rect.size.width {
                let point = Point::new(
                    rect.top_left.x + x as i32,
                    rect.top_left.y + y as i32,
                );
                let transformed = self.transform.transform_point(point);
                let size = target.size();
                
                if transformed.x >= 0 && transformed.x < size.width as i32 
                    && transformed.y >= 0 && transformed.y < size.height as i32 {
                    target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                }
            }
        }
        
        Ok(())
    }
    
    /// 描边矩形
    fn stroke_rectangle<T: RenderTarget>(
        &self,
        target: &mut T,
        rect: Rectangle,
    ) -> Result<(), GraphicsError> {
        let top_left = rect.top_left;
        let top_right = Point::new(
            rect.top_left.x + rect.size.width as i32 - 1,
            rect.top_left.y,
        );
        let bottom_left = Point::new(
            rect.top_left.x,
            rect.top_left.y + rect.size.height as i32 - 1,
        );
        let bottom_right = Point::new(
            rect.top_left.x + rect.size.width as i32 - 1,
            rect.top_left.y + rect.size.height as i32 - 1,
        );
        
        // 绘制四条边
        self.draw_line(target, top_left, top_right)?;
        self.draw_line(target, top_right, bottom_right)?;
        self.draw_line(target, bottom_right, bottom_left)?;
        self.draw_line(target, bottom_left, top_left)?;
        
        Ok(())
    }
    
    /// 绘制圆形
    pub fn draw_circle<T: RenderTarget>(
        &self,
        target: &mut T,
        center: Point,
        radius: u32,
    ) -> Result<(), GraphicsError> {
        match self.draw_params.draw_style {
            DrawStyle::Fill => self.fill_circle(target, center, radius),
            DrawStyle::Stroke => self.stroke_circle(target, center, radius),
            DrawStyle::FillStroke => {
                self.fill_circle(target, center, radius)?;
                self.stroke_circle(target, center, radius)
            }
        }
    }
    
    /// 填充圆形
    fn fill_circle<T: RenderTarget>(
        &self,
        target: &mut T,
        center: Point,
        radius: u32,
    ) -> Result<(), GraphicsError> {
        let color = self.draw_params.fill_color.unwrap_or(Color::WHITE);
        let r = radius as i32;
        
        for y in -r..=r {
            for x in -r..=r {
                if x * x + y * y <= r * r {
                    let point = Point::new(center.x + x, center.y + y);
                    let transformed = self.transform.transform_point(point);
                    let size = target.size();
                    
                    if transformed.x >= 0 && transformed.x < size.width as i32 
                        && transformed.y >= 0 && transformed.y < size.height as i32 {
                        target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// 描边圆形（Bresenham圆算法）
    fn stroke_circle<T: RenderTarget>(
        &self,
        target: &mut T,
        center: Point,
        radius: u32,
    ) -> Result<(), GraphicsError> {
        let color = self.draw_params.stroke_color.unwrap_or(Color::BLACK);
        let width = self.draw_params.stroke_width;
        
        // Bresenham圆算法
        let mut x = 0i32;
        let mut y = radius as i32;
        let mut d = 3 - 2 * radius as i32;
        
        while y >= x {
            // 绘制8个对称点
            self.draw_circle_points(target, center, x, y, color, width)?;
            
            x += 1;
            if d > 0 {
                y -= 1;
                d = d + 4 * (x - y) + 10;
            } else {
                d = d + 4 * x + 6;
            }
        }
        
        Ok(())
    }
    
    /// 绘制圆的8个对称点
    fn draw_circle_points<T: RenderTarget>(
        &self,
        target: &mut T,
        center: Point,
        x: i32,
        y: i32,
        color: Color,
        width: u8,
    ) -> Result<(), GraphicsError> {
        let points = [
            Point::new(center.x + x, center.y + y),
            Point::new(center.x - x, center.y + y),
            Point::new(center.x + x, center.y - y),
            Point::new(center.x - x, center.y - y),
            Point::new(center.x + y, center.y + x),
            Point::new(center.x - y, center.y + x),
            Point::new(center.x + y, center.y - x),
            Point::new(center.x - y, center.y - x),
        ];
        
        for point in &points {
            if width > 1 {
                let half_width = width as i32 / 2;
                for dy in -half_width..=half_width {
                    for dx in -half_width..=half_width {
                        let px = point.x + dx;
                        let py = point.y + dy;
                        let transformed = self.transform.transform_point(Point::new(px, py));
                        let size = target.size();
                        
                        if transformed.x >= 0 && transformed.x < size.width as i32 
                            && transformed.y >= 0 && transformed.y < size.height as i32 {
                            target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                        }
                    }
                }
            } else {
                let transformed = self.transform.transform_point(*point);
                let size = target.size();
                
                if transformed.x >= 0 && transformed.x < size.width as i32 
                    && transformed.y >= 0 && transformed.y < size.height as i32 {
                    target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                }
            }
        }
        
        Ok(())
    }
    
    /// 绘制椭圆
    pub fn draw_ellipse<T: RenderTarget>(
        &self,
        target: &mut T,
        center: Point,
        rx: u32,
        ry: u32,
    ) -> Result<(), GraphicsError> {
        match self.draw_params.draw_style {
            DrawStyle::Fill => self.fill_ellipse(target, center, rx, ry),
            DrawStyle::Stroke => self.stroke_ellipse(target, center, rx, ry),
            DrawStyle::FillStroke => {
                self.fill_ellipse(target, center, rx, ry)?;
                self.stroke_ellipse(target, center, rx, ry)
            }
        }
    }
    
    /// 填充椭圆
    fn fill_ellipse<T: RenderTarget>(
        &self,
        target: &mut T,
        center: Point,
        rx: u32,
        ry: u32,
    ) -> Result<(), GraphicsError> {
        let color = self.draw_params.fill_color.unwrap_or(Color::WHITE);
        let rx_sq = (rx * rx) as f32;
        let ry_sq = (ry * ry) as f32;
        
        for y in -(ry as i32)..=(ry as i32) {
            for x in -(rx as i32)..=(rx as i32) {
                let x_norm = x as f32 / rx as f32;
                let y_norm = y as f32 / ry as f32;
                
                if x_norm * x_norm + y_norm * y_norm <= 1.0 {
                    let point = Point::new(center.x + x, center.y + y);
                    let transformed = self.transform.transform_point(point);
                    let size = target.size();
                    
                    if transformed.x >= 0 && transformed.x < size.width as i32 
                        && transformed.y >= 0 && transformed.y < size.height as i32 {
                        target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// 描边椭圆
    fn stroke_ellipse<T: RenderTarget>(
        &self,
        target: &mut T,
        center: Point,
        rx: u32,
        ry: u32,
    ) -> Result<(), GraphicsError> {
        let color = self.draw_params.stroke_color.unwrap_or(Color::BLACK);
        
        // 使用参数方程绘制椭圆
        let steps = (2.0 * core::f32::consts::PI * rx.max(ry) as f32) as u32;
        let angle_step = 2.0 * core::f32::consts::PI / steps as f32;
        
        let mut prev_point = None;
        
        for i in 0..=steps {
            let angle = i as f32 * angle_step;
            let x = center.x + (rx as f32 * angle.cos()) as i32;
            let y = center.y + (ry as f32 * angle.sin()) as i32;
            let point = Point::new(x, y);
            
            if let Some(prev) = prev_point {
                self.draw_line(target, prev, point)?;
            }
            prev_point = Some(point);
        }
        
        Ok(())
    }
    
    /// 绘制三角形
    pub fn draw_triangle<T: RenderTarget>(
        &self,
        target: &mut T,
        p1: Point,
        p2: Point,
        p3: Point,
    ) -> Result<(), GraphicsError> {
        match self.draw_params.draw_style {
            DrawStyle::Fill => self.fill_triangle(target, p1, p2, p3),
            DrawStyle::Stroke => self.stroke_triangle(target, p1, p2, p3),
            DrawStyle::FillStroke => {
                self.fill_triangle(target, p1, p2, p3)?;
                self.stroke_triangle(target, p1, p2, p3)
            }
        }
    }
    
    /// 填充三角形
    fn fill_triangle<T: RenderTarget>(
        &self,
        target: &mut T,
        p1: Point,
        p2: Point,
        p3: Point,
    ) -> Result<(), GraphicsError> {
        let color = self.draw_params.fill_color.unwrap_or(Color::WHITE);
        
        // 计算边界框
        let min_x = p1.x.min(p2.x).min(p3.x);
        let max_x = p1.x.max(p2.x).max(p3.x);
        let min_y = p1.y.min(p2.y).min(p3.y);
        let max_y = p1.y.max(p2.y).max(p3.y);
        
        // 使用重心坐标判断点是否在三角形内
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let point = Point::new(x, y);
                if self.point_in_triangle(point, p1, p2, p3) {
                    let transformed = self.transform.transform_point(point);
                    let size = target.size();
                    
                    if transformed.x >= 0 && transformed.x < size.width as i32 
                        && transformed.y >= 0 && transformed.y < size.height as i32 {
                        target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// 描边三角形
    fn stroke_triangle<T: RenderTarget>(
        &self,
        target: &mut T,
        p1: Point,
        p2: Point,
        p3: Point,
    ) -> Result<(), GraphicsError> {
        self.draw_line(target, p1, p2)?;
        self.draw_line(target, p2, p3)?;
        self.draw_line(target, p3, p1)?;
        Ok(())
    }
    
    /// 判断点是否在三角形内（重心坐标法）
    fn point_in_triangle(&self, p: Point, a: Point, b: Point, c: Point) -> bool {
        let denom = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
        if denom == 0 {
            return false; // 退化三角形
        }
        
        let alpha = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) as f32 / denom as f32;
        let beta = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) as f32 / denom as f32;
        let gamma = 1.0 - alpha - beta;
        
        alpha >= 0.0 && beta >= 0.0 && gamma >= 0.0
    }
    
    /// 绘制多边形
    pub fn draw_polygon<T: RenderTarget>(
        &self,
        target: &mut T,
        points: &[Point],
    ) -> Result<(), GraphicsError> {
        if points.len() < 3 {
            return Err(GraphicsError::InvalidParameter);
        }
        
        match self.draw_params.draw_style {
            DrawStyle::Fill => self.fill_polygon(target, points),
            DrawStyle::Stroke => self.stroke_polygon(target, points),
            DrawStyle::FillStroke => {
                self.fill_polygon(target, points)?;
                self.stroke_polygon(target, points)
            }
        }
    }
    
    /// 填充多边形（扫描线算法）
    fn fill_polygon<T: RenderTarget>(
        &self,
        target: &mut T,
        points: &[Point],
    ) -> Result<(), GraphicsError> {
        let color = self.draw_params.fill_color.unwrap_or(Color::WHITE);
        
        // 计算边界框
        let min_y = points.iter().map(|p| p.y).min().unwrap();
        let max_y = points.iter().map(|p| p.y).max().unwrap();
        
        // 扫描线填充
        for y in min_y..=max_y {
            let mut intersections: Vec<i32, 32> = Vec::new();
            
            // 计算与扫描线的交点
            for i in 0..points.len() {
                let j = (i + 1) % points.len();
                let p1 = points[i];
                let p2 = points[j];
                
                if (p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y) {
                    let x = p1.x + (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
                    let _ = intersections.push(x);
                }
            }
            
            // 排序交点
            intersections.sort();
            
            // 填充交点对之间的区域
            for chunk in intersections.chunks(2) {
                if chunk.len() == 2 {
                    for x in chunk[0]..=chunk[1] {
                        let point = Point::new(x, y);
                        let transformed = self.transform.transform_point(point);
                        let size = target.size();
                        
                        if transformed.x >= 0 && transformed.x < size.width as i32 
                            && transformed.y >= 0 && transformed.y < size.height as i32 {
                            target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                        }
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// 描边多边形
    fn stroke_polygon<T: RenderTarget>(
        &self,
        target: &mut T,
        points: &[Point],
    ) -> Result<(), GraphicsError> {
        for i in 0..points.len() {
            let j = (i + 1) % points.len();
            self.draw_line(target, points[i], points[j])?;
        }
        Ok(())
    }
    
    /// 绘制贝塞尔曲线
    pub fn draw_bezier_curve<T: RenderTarget>(
        &self,
        target: &mut T,
        p0: Point,
        p1: Point,
        p2: Point,
        p3: Point,
        segments: u32,
    ) -> Result<(), GraphicsError> {
        let mut prev_point = p0;
        
        for i in 1..=segments {
            let t = i as f32 / segments as f32;
            let point = self.cubic_bezier_point(p0, p1, p2, p3, t);
            self.draw_line(target, prev_point, point)?;
            prev_point = point;
        }
        
        Ok(())
    }
    
    /// 计算三次贝塞尔曲线上的点
    fn cubic_bezier_point(&self, p0: Point, p1: Point, p2: Point, p3: Point, t: f32) -> Point {
        let u = 1.0 - t;
        let tt = t * t;
        let uu = u * u;
        let uuu = uu * u;
        let ttt = tt * t;
        
        let x = uuu * p0.x as f32 + 3.0 * uu * t * p1.x as f32 + 3.0 * u * tt * p2.x as f32 + ttt * p3.x as f32;
        let y = uuu * p0.y as f32 + 3.0 * uu * t * p1.y as f32 + 3.0 * u * tt * p2.y as f32 + ttt * p3.y as f32;
        
        Point::new(x as i32, y as i32)
    }
    
    /// 绘制图像
    pub fn draw_image<T: RenderTarget>(
        &self,
        target: &mut T,
        image: &ImageData,
        position: Point,
    ) -> Result<(), GraphicsError> {
        for y in 0..image.info.height {
            for x in 0..image.info.width {
                if let Ok(color) = image.get_pixel(x, y) {
                    let point = Point::new(
                        position.x + x as i32,
                        position.y + y as i32,
                    );
                    let transformed = self.transform.transform_point(point);
                    let size = target.size();
                    
                    if transformed.x >= 0 && transformed.x < size.width as i32 
                        && transformed.y >= 0 && transformed.y < size.height as i32 {
                        // 处理透明度
                        if color.a < 255 {
                            if let Ok(bg_color) = target.get_pixel(transformed.x as u16, transformed.y as u16) {
                                let blended = bg_color.blend(&color, color.a);
                                target.set_pixel(transformed.x as u16, transformed.y as u16, blended)?;
                            } else {
                                target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                            }
                        } else {
                            target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                        }
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// 绘制缩放图像
    pub fn draw_scaled_image<T: RenderTarget>(
        &self,
        target: &mut T,
        image: &ImageData,
        src_rect: Rectangle,
        dst_rect: Rectangle,
    ) -> Result<(), GraphicsError> {
        let x_scale = src_rect.size.width as f32 / dst_rect.size.width as f32;
        let y_scale = src_rect.size.height as f32 / dst_rect.size.height as f32;
        
        for dst_y in 0..dst_rect.size.height {
            for dst_x in 0..dst_rect.size.width {
                let src_x = (dst_x as f32 * x_scale) as u16 + src_rect.top_left.x as u16;
                let src_y = (dst_y as f32 * y_scale) as u16 + src_rect.top_left.y as u16;
                
                if src_x < image.info.width && src_y < image.info.height {
                    if let Ok(color) = image.get_pixel(src_x, src_y) {
                        let point = Point::new(
                            dst_rect.top_left.x + dst_x as i32,
                            dst_rect.top_left.y + dst_y as i32,
                        );
                        let transformed = self.transform.transform_point(point);
                        let size = target.size();
                        
                        if transformed.x >= 0 && transformed.x < size.width as i32 
                            && transformed.y >= 0 && transformed.y < size.height as i32 {
                            target.set_pixel(transformed.x as u16, transformed.y as u16, color)?;
                        }
                    }
                }
            }
        }
        
        Ok(())
    }
}

impl Default for GraphicsRenderer {
    fn default() -> Self {
        Self::new()
    }
}

/// 渲染统计信息
#[derive(Debug, Clone, Default)]
pub struct RenderStats {
    /// 绘制的像素数
    pub pixels_drawn: u32,
    /// 绘制的图形数
    pub primitives_drawn: u32,
    /// 渲染时间（微秒）
    pub render_time_us: u32,
    /// 内存使用量（字节）
    pub memory_used: u32,
}

impl RenderStats {
    /// 创建新的统计信息
    pub fn new() -> Self {
        Self::default()
    }
    
    /// 重置统计信息
    pub fn reset(&mut self) {
        *self = Self::default();
    }
    
    /// 添加像素数
    pub fn add_pixels(&mut self, count: u32) {
        self.pixels_drawn += count;
    }
    
    /// 添加图形数
    pub fn add_primitive(&mut self) {
        self.primitives_drawn += 1;
    }
    
    /// 设置渲染时间
    pub fn set_render_time(&mut self, time_us: u32) {
        self.render_time_us = time_us;
    }
    
    /// 设置内存使用量
    pub fn set_memory_used(&mut self, memory: u32) {
        self.memory_used = memory;
    }
    
    /// 计算每秒像素数
    pub fn pixels_per_second(&self) -> f32 {
        if self.render_time_us > 0 {
            self.pixels_drawn as f32 / (self.render_time_us as f32 / 1_000_000.0)
        } else {
            0.0
        }
    }
    
    /// 计算每秒图形数
    pub fn primitives_per_second(&self) -> f32 {
        if self.render_time_us > 0 {
            self.primitives_drawn as f32 / (self.render_time_us as f32 / 1_000_000.0)
        } else {
            0.0
        }
    }
}