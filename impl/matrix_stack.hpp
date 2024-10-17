#pragma once

#include "mat43.hpp"

#include <stack>

namespace SaturnMath
{
    class MatrixStack : public std::stack<Mat43>
    {
    public:
        /**
         * @brief Get the top matrix from the stack.
         * @return The top matrix on the stack.
         */
        Mat43 Top()
        {
            if (!this->empty())
            {
                return this->top();
            }

            // If the stack is empty, return the identity matrix.
            return Mat43::Identity();
        }

        /**
         * @brief Get the combined matrix from the stack.
         * @return The combined matrix.
         */
        Mat43 GetCombinedMatrix()
        {
            std::stack<Mat43> tempStack = *this;
            Mat43 combinedMatrix = Mat43::Identity();

            while (!tempStack.empty())
            {
                combinedMatrix = combinedMatrix * tempStack.top();
                tempStack.pop();
            }

            return combinedMatrix;
        }

        /**
         * @brief Clear the matrix stack.
         */
        void Clear()
        {
            while (!this->empty())
            {
                this->pop();
            }
        }
    };
}